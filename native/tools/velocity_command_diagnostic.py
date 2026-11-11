#!/usr/bin/env python3
"""Observe VelocityCommand without creating any flight-control output.

ROS imports live in main; the receiver and bounded file writer need no ROS graph.
Zero-valued limits mean unconfigured, never an implicit flight qualification.
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import time


MESSAGE_TYPE = 'jikl_fc01_system/msg/VelocityCommand'
DEFAULT_TOPIC = '/fc01/vision/velocity_command'
MAX_TEXT = 256
STREAM_HISTORY_CAPACITY = 32
LIMIT_NAMES = ('timeout_sec', 'max_speed_mps', 'max_source_age_sec', 'future_tolerance_sec')


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _integer(value, maximum):
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= maximum


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _text(value):
    return value[:MAX_TEXT] if isinstance(value, str) else None


@dataclass(frozen=True)
class DiagnosticConfig:
    timeout_sec: float = 0.0
    max_speed_mps: float = 0.0
    max_source_age_sec: float = 0.0
    future_tolerance_sec: float = 0.0
    expected_source_id: str = ''

    def __post_init__(self):
        for name in LIMIT_NAMES:
            value = getattr(self, name)
            if not _finite(value) or value < 0:
                raise ValueError(name + ' must be finite and nonnegative; 0 means unconfigured')
        if (not isinstance(self.expected_source_id, str) or len(self.expected_source_id) > MAX_TEXT
                or (self.expected_source_id and not self.expected_source_id.strip())):
            raise ValueError('expected_source_id must be empty (unbound) or an explicit source identifier')


class Receiver:
    """Diagnostic observations only; accepted means format/order checks passed.

    No observation from this class authorizes movement. The original message and
    the latest anomaly remain separate so good traffic cannot erase an incident.
    """
    def __init__(self, config=None):
        self.config = config or DiagnosticConfig()
        self.counters = dict.fromkeys(('received', 'accepted', 'invalid', 'duplicate',
                                      'out_of_order', 'source_change', 'reset_change',
                                      'source_count', 'timeout', 'clock_regression',
                                      'source_expired', 'anomalies', 'events_dropped'), 0)
        self.last_message = None
        self.last_anomaly = None
        self.last_event = None
        self.latest_errors = []
        self.publisher_count = None
        self._identity = None
        self._watermarks = OrderedDict()
        self._sequence = 0
        self._stamp = 0
        self._last_valid = None
        self._candidate = None
        self._last_observation_steady = None
        self._last_observation_ros = None
        self._timeout_reported = False
        self._events = deque(maxlen=128)
        self._event_id = 0

    def _observe_clocks(self, now, ros_now_ns):
        errors = []
        if self._last_observation_steady is not None and now < self._last_observation_steady:
            errors.append('RECEIVE_CLOCK_REGRESSION')
        if (self._last_observation_ros is not None and ros_now_ns
                and ros_now_ns < self._last_observation_ros):
            errors.append('ROS_CLOCK_REGRESSION')
        self._last_observation_steady = now
        if ros_now_ns:
            self._last_observation_ros = ros_now_ns
        if errors:
            self._candidate = None
            self.counters['clock_regression'] += 1
            self._event('clock_regression', errors, now, ros_now_ns)
        return errors

    def _observe_receive_timeout(self, now, ros_now_ns):
        # A healthy callback may arrive after an executor pause. Observe the
        # old lifetime before replacing its receipt time, so the gap remains
        # evidence even when no periodic snapshot ran during the interruption.
        if (self._last_valid is None or not self.config.timeout_sec
                or now - self._last_valid['received_at'] < self.config.timeout_sec):
            return
        self._candidate = None
        if not self._timeout_reported:
            self._timeout_reported = True
            self.counters['timeout'] += 1
            self._event('timeout', ['RECEIVE_TIMEOUT'], now, ros_now_ns,
                        last_valid_age_sec=now - self._last_valid['received_at'],
                        last_valid_sequence=self._last_valid['sequence'])

    def _event(self, kind, reasons, now, ros_now_ns, **fields):
        self._event_id += 1
        event = _json_safe(dict(event_id=self._event_id, kind=kind, reasons=list(reasons),
                               received_at=now, ros_now_ns=ros_now_ns,
                               control_authority=False, **fields))
        self.last_event = event
        if reasons:
            self.last_anomaly = event
            self.counters['anomalies'] += 1
        if len(self._events) == self._events.maxlen:
            self.counters['events_dropped'] += 1
        self._events.append(event)
        return event

    def drain_events(self):
        result = list(self._events)
        self._events.clear()
        return result

    def observe_publishers(self, count, now, ros_now_ns):
        if not _integer(count, 2**32 - 1):
            raise ValueError('publisher_count must be a nonnegative integer')
        if count == self.publisher_count:
            return None
        previous = self.publisher_count
        self.publisher_count = count
        reasons = [] if count == 1 else ['PUBLISHER_COUNT_NOT_ONE']
        if reasons:
            self._candidate = None
            self.counters['source_count'] += 1
        return self._event('publisher_count', reasons, now, ros_now_ns,
                           previous_count=previous, publisher_count=count)

    def receive(self, message, received_at, ros_now_ns, publisher_count):
        if not _finite(received_at) or received_at < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('independent receive clocks must be finite and nonnegative')
        clock_errors = self._observe_clocks(received_at, ros_now_ns)
        self._observe_receive_timeout(received_at, ros_now_ns)
        self.observe_publishers(publisher_count, received_at, ros_now_ns)
        self.counters['received'] += 1
        stamp = getattr(message, 'stamp', None)
        sec, nanosec = getattr(stamp, 'sec', None), getattr(stamp, 'nanosec', None)
        stamp_ok = _integer(sec, 2**31 - 1) and _integer(nanosec, 999999999)
        stamp_ns = sec * 1000000000 + nanosec if stamp_ok else 0
        schema = getattr(message, 'schema_version', None)
        sequence = getattr(message, 'sequence', None)
        reset = getattr(message, 'reset_counter', None)
        source = getattr(message, 'source_id', None)
        frame = getattr(message, 'frame_id', None)
        valid = getattr(message, 'valid', None)
        velocities = [getattr(message, name, None) for name in ('vx', 'vy', 'vz')]
        source_age = (ros_now_ns - stamp_ns) / 1e9 if stamp_ns and ros_now_ns else None
        sample = dict(schema_version=schema, sequence=sequence,
                      stamp=dict(sec=sec, nanosec=nanosec), stamp_ns=stamp_ns or None,
                      reset_counter=reset, source_id=_text(source), frame_id=_text(frame),
                      valid=valid, vx=velocities[0], vy=velocities[1], vz=velocities[2],
                      received_at=received_at, source_age_at_receive_sec=source_age,
                      publisher_count=publisher_count)
        errors = list(clock_errors)
        if schema != 1 or isinstance(schema, bool):
            errors.append('SCHEMA_VERSION_INVALID')
        if not _integer(sequence, 2**64 - 1) or not sequence:
            errors.append('SEQUENCE_INVALID')
        if not stamp_ok or not stamp_ns:
            errors.append('STAMP_INVALID')
        if not _integer(reset, 2**32 - 1):
            errors.append('RESET_COUNTER_INVALID')
        if not isinstance(source, str) or not source.strip() or len(source) > MAX_TEXT:
            errors.append('SOURCE_ID_INVALID')
        elif self.config.expected_source_id and source != self.config.expected_source_id:
            errors.append('SOURCE_ID_NOT_EXPECTED')
        if frame != 'body_frd':
            errors.append('FRAME_ID_MUST_BE_EXPLICIT_BODY_FRD')
        if valid is not True:
            errors.append('COMMAND_INVALID')
        if not all(_finite(value) for value in velocities):
            errors.append('VELOCITY_NONFINITE_OR_MISSING')
        elif self.config.max_speed_mps and math.hypot(*velocities) > self.config.max_speed_mps:
            errors.append('SPEED_LIMIT_EXCEEDED')
        if publisher_count != 1:
            errors.append('PUBLISHER_COUNT_NOT_ONE')
        if source_age is not None:
            if self.config.max_source_age_sec and source_age >= self.config.max_source_age_sec:
                errors.append('SOURCE_AGE_LIMIT_EXCEEDED')
            if self.config.future_tolerance_sec and source_age < -self.config.future_tolerance_sec:
                errors.append('SOURCE_STAMP_IN_FUTURE')
        elif self.config.max_source_age_sec or self.config.future_tolerance_sec:
            errors.append('ROS_CLOCK_UNAVAILABLE')

        identity_valid = (_integer(reset, 2**32 - 1) and isinstance(source, str)
                          and bool(source.strip()) and len(source) <= MAX_TEXT)
        if identity_valid:
            identity = (source, reset)
            if self._identity is not None and identity != self._identity:
                changes = []
                if source != self._identity[0]:
                    self.counters['source_change'] += 1
                    changes.append('SOURCE_CHANGED')
                if reset != self._identity[1]:
                    self.counters['reset_change'] += 1
                    changes.append('RESET_CHANGED')
                self._event('stream_change', changes, received_at, ros_now_ns,
                            previous_source_id=self._identity[0], previous_reset_counter=self._identity[1],
                            source_id=source, reset_counter=reset)
                self._candidate = None
            self._identity = identity
            self._sequence, self._stamp = self._watermarks.get(identity, (0, 0))
            source_matches = not self.config.expected_source_id or source == self.config.expected_source_id
            if (source_matches and identity not in self._watermarks
                    and len(self._watermarks) >= STREAM_HISTORY_CAPACITY):
                # Retain replay evidence rather than evicting an old stream.
                # A new standalone diagnostic process can start a fresh audit;
                # existing identities may still deliver healthy new samples.
                errors.append('STREAM_HISTORY_CAPACITY_REACHED')
            if _integer(sequence, 2**64 - 1) and sequence:
                if sequence == self._sequence:
                    errors.append('DUPLICATE_SEQUENCE')
                    self.counters['duplicate'] += 1
                elif sequence < self._sequence:
                    errors.append('OUT_OF_ORDER_SEQUENCE')
                    self.counters['out_of_order'] += 1
                else:
                    source_stamp = max((watermark[1] for key, watermark in self._watermarks.items()
                                        if key[0] == source), default=0)
                    if source_stamp > self._stamp and stamp_ns and stamp_ns <= source_stamp:
                        # reset_counter restarts the sender epoch, not the
                        # agreed ROS/system measurement clock. Preserve the
                        # same-source stamp across its bounded epoch history.
                        errors.append('SOURCE_STAMP_NOT_ADVANCING_ACROSS_RESET')
                    self._sequence = sequence
                    if stamp_ns and stamp_ns <= self._stamp:
                        errors.append('STAMP_NOT_ADVANCING')
                        if stamp_ns < self._stamp:
                            errors.append('SOURCE_CLOCK_REGRESSION')
                            self.counters['clock_regression'] += 1
                    if stamp_ns:
                        self._stamp = max(self._stamp, stamp_ns)
                # Rejected clocks, schemas, frames or numeric payloads must not
                # poison trusted sequence/stamp watermarks. A well-formed false
                # validity flag is an explicit withdrawal, so its identity is
                # consumed and cannot be replayed as a valid command later.
                advance = not errors or (errors == ['COMMAND_INVALID'] and valid is False)
                if advance and source_matches:
                    self._watermarks[identity] = (self._sequence, self._stamp)
        accepted = not errors
        if accepted:
            self.counters['accepted'] += 1
            self._last_valid = sample.copy()
            self._candidate = sample.copy()
            self._timeout_reported = False
        else:
            self._candidate = None
            self.counters['invalid'] += 1
        self.last_message = _json_safe(sample)
        self.latest_errors = errors
        return self._event('message', errors, received_at, ros_now_ns,
                           accepted=accepted, sample=sample)

    def snapshot(self, now, ros_now_ns, publisher_count=None):
        if not _finite(now) or now < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('snapshot clocks must be finite and nonnegative')
        self._observe_clocks(now, ros_now_ns)
        self._observe_receive_timeout(now, ros_now_ns)
        if publisher_count is not None:
            self.observe_publishers(publisher_count, now, ros_now_ns)
        age = now - self._last_valid['received_at'] if self._last_valid else None
        if not self.config.timeout_sec:
            receive_freshness = 'UNCONFIGURED'
        elif age is None:
            receive_freshness = 'NO_SAMPLE'
        elif age < 0:
            receive_freshness = 'CLOCK_REGRESSION'
        elif age >= self.config.timeout_sec:
            receive_freshness = 'EXPIRED'
        else:
            receive_freshness = 'WITHIN_RECEIVE_LIMIT'
        last = dict(self.last_message) if self.last_message is not None else None
        if last is not None:
            last['receive_age_sec'] = now - last['received_at']
            last['source_age_sec'] = ((ros_now_ns - last['stamp_ns']) / 1e9
                                      if ros_now_ns and last['stamp_ns'] else None)
        candidate_errors = []
        if self._candidate is not None:
            candidate_age = now - self._candidate['received_at']
            source_age = ((ros_now_ns - self._candidate['stamp_ns']) / 1e9
                          if ros_now_ns and self._candidate['stamp_ns'] else None)
            if candidate_age < 0:
                candidate_errors.append('RECEIVE_CLOCK_REGRESSION')
            elif self.config.timeout_sec and candidate_age >= self.config.timeout_sec:
                candidate_errors.append('RECEIVE_TIMEOUT')
            if source_age is None:
                candidate_errors.append('ROS_CLOCK_UNAVAILABLE')
            elif self.config.max_source_age_sec and source_age >= self.config.max_source_age_sec:
                candidate_errors.append('SOURCE_AGE_LIMIT_EXCEEDED')
            elif self.config.future_tolerance_sec and source_age < -self.config.future_tolerance_sec:
                candidate_errors.append('SOURCE_STAMP_IN_FUTURE')
            if candidate_errors:
                self._candidate = None
                self.counters['source_expired'] += 1
                self._event('cache_revoked', candidate_errors, now, ros_now_ns)
        if self._candidate is None:
            candidate_errors.append('NO_CURRENT_CANDIDATE')
        if not all(getattr(self.config, name) > 0 for name in LIMIT_NAMES):
            candidate_errors.append('LIMITS_UNCONFIGURED')
        if not self.config.expected_source_id:
            candidate_errors.append('SOURCE_BINDING_UNCONFIGURED')
        if self.publisher_count != 1:
            candidate_errors.append('PUBLISHER_COUNT_NOT_ONE')
        freshness = 'UNCONFIGURED'
        if receive_freshness in ('EXPIRED', 'CLOCK_REGRESSION', 'NO_SAMPLE'):
            freshness = receive_freshness
        elif self.latest_errors:
            freshness = 'LATEST_MESSAGE_INVALID'
        elif all((self.config.timeout_sec, self.config.max_source_age_sec,
                  self.config.future_tolerance_sec)) and last is not None:
            source_age = last['source_age_sec']
            if source_age is None:
                freshness = 'ROS_CLOCK_UNAVAILABLE'
            elif source_age >= self.config.max_source_age_sec:
                freshness = 'EXPIRED'
            elif source_age < -self.config.future_tolerance_sec:
                freshness = 'SOURCE_STAMP_IN_FUTURE'
            else:
                freshness = 'CURRENT'
        return _json_safe(dict(schema_version=1, kind='velocity_command_diagnostic',
                               message_type=MESSAGE_TYPE, control_authority=False,
                               qualification='DIAGNOSTIC_ONLY',
                               limits={name: getattr(self.config, name) or None for name in LIMIT_NAMES},
                               expected_source_id=self.config.expected_source_id or None,
                               stream_history_size=len(self._watermarks),
                               stream_history_capacity=STREAM_HISTORY_CAPACITY,
                               cache_scope='DIAGNOSTIC_ONLY', cache_usable=not candidate_errors,
                               cache_errors=candidate_errors,
                               current_candidate=self._candidate if not candidate_errors else None,
                               historical_last_accepted_message=self._last_valid,
                               observed_at=now, ros_now_ns=ros_now_ns,
                               publisher_count=self.publisher_count, freshness=freshness,
                               receive_freshness=receive_freshness,
                               last_valid_age_sec=age, last_valid_is_historical=True, last_message=last,
                               latest_errors=list(self.latest_errors),
                               last_anomaly=self.last_anomaly, last_event=self.last_event,
                               counters=dict(self.counters)))


class DiagnosticWriter:
    """Atomic snapshot plus a bounded, rate-limited observation log."""
    def __init__(self, output_dir='', log_max_bytes=262144, log_backup_count=1,
                 log_min_interval_sec=0.2):
        if not _integer(log_max_bytes, 16 * 1024 * 1024) or log_max_bytes < 1024:
            raise ValueError('log_max_bytes must be an integer in [1024,16777216]')
        if not _integer(log_backup_count, 10):
            raise ValueError('log_backup_count must be an integer in [0,10]')
        if not _finite(log_min_interval_sec) or log_min_interval_sec < 0:
            raise ValueError('log_min_interval_sec must be finite and nonnegative')
        self.directory = Path(output_dir).expanduser() if output_dir else None
        self.max_bytes, self.backups = log_max_bytes, log_backup_count
        self.min_interval = log_min_interval_sec
        self.last_logged_at = None
        self.suppressed = 0
        self._pending_anomaly = None
        if self.directory is not None:
            self.directory.mkdir(parents=True, exist_ok=True)

    def record(self, event, now):
        if self.directory is None:
            return False
        if self._pending_anomaly is not None:
            self.flush(now)
        if self.last_logged_at is not None and 0 <= now - self.last_logged_at < self.min_interval:
            self.suppressed += 1
            if event.get('reasons'):
                if self._pending_anomaly is None:
                    self._pending_anomaly = dict(event, coalesced_anomalies=1)
                else:
                    self._pending_anomaly['coalesced_anomalies'] += 1
                    reasons = self._pending_anomaly.setdefault('coalesced_reasons', [])
                    for reason in event['reasons']:
                        if reason not in reasons and len(reasons) < 64:
                            reasons.append(reason)
            return False
        return self._write(event, now)

    def flush(self, now):
        if self._pending_anomaly is None:
            return False
        if self.last_logged_at is not None and 0 <= now - self.last_logged_at < self.min_interval:
            return False
        pending, self._pending_anomaly = self._pending_anomaly, None
        return self._write(pending, now)

    def _write(self, event, now):
        value = dict(event, log_events_suppressed=self.suppressed)
        data = (json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False) + '\n').encode()
        if len(data) > self.max_bytes:
            data = (json.dumps(dict(kind='event_truncated', event_id=event.get('event_id'),
                                    reasons=['LOG_RECORD_TOO_LARGE'], control_authority=False,
                                    log_events_suppressed=self.suppressed)) + '\n').encode()
        path = self.directory / 'velocity_command_diagnostic.log'
        if path.exists() and path.stat().st_size + len(data) > self.max_bytes:
            for number in range(self.backups, 0, -1):
                source = path if number == 1 else Path(str(path) + '.' + str(number - 1))
                destination = Path(str(path) + '.' + str(number))
                if source.exists():
                    os.replace(source, destination)
            if not self.backups:
                path.unlink()
        with path.open('ab') as stream:
            stream.write(data)
        self.last_logged_at = now
        self.suppressed = 0
        return True

    def snapshot(self, value):
        if self.directory is None:
            return
        data = json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        descriptor, temporary = tempfile.mkstemp(prefix='.velocity_command_diagnostic.', dir=self.directory)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                stream.write(data)
            os.replace(temporary, self.directory / 'velocity_command_diagnostic.json')
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from jikl_fc01_system.msg import VelocityCommand

    class DiagnosticNode(Node):
        def __init__(self):
            super().__init__('fc01_velocity_command_diagnostic')

            def parameter(name, default):
                return self.declare_parameter(name, default).value

            self.topic = parameter('topic', DEFAULT_TOPIC)
            depth = parameter('qos_depth', 5)
            reliable = parameter('reliable', False)
            period = parameter('snapshot_period_sec', 1.0)
            if (not isinstance(self.topic, str) or not self.topic.startswith('/')
                    or self.topic.startswith('/fmu/') or not _integer(depth, 20) or depth < 1
                    or not _finite(period) or period < .01 or period > 60):
                raise ValueError('invalid diagnostic topic, depth, or snapshot period')
            self.receiver = Receiver(DiagnosticConfig(**{
                name: parameter(name, default) for name, default in vars(DiagnosticConfig()).items()}))
            self.writer = DiagnosticWriter(parameter('output_dir', ''),
                                           parameter('log_max_bytes', 262144),
                                           parameter('log_backup_count', 1),
                                           parameter('log_min_interval_sec', .2))
            qos = QoSProfile(depth=depth, durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
            self.subscription = self.create_subscription(VelocityCommand, self.topic, self.receive, qos)
            self.timer = self.create_timer(period, self.snapshot)
            self.get_logger().info('Diagnostic only; control_authority=false type=' + MESSAGE_TYPE +
                                   ' topic=' + self.topic + ' zero limits are UNCONFIGURED')
            self.snapshot()

        def receive_context(self):
            return time.monotonic(), max(0, self.get_clock().now().nanoseconds), self.count_publishers(self.topic)

        def flush_events(self, now):
            for event in self.receiver.drain_events():
                self.writer.record(event, now)

        def receive(self, message):
            now, ros_now, publishers = self.receive_context()
            self.receiver.receive(message, now, ros_now, publishers)
            self.flush_events(now)

        def snapshot(self):
            now, ros_now, publishers = self.receive_context()
            value = self.receiver.snapshot(now, ros_now, publishers)
            value['topic'] = self.topic
            value['log_events_suppressed'] = self.writer.suppressed
            self.flush_events(now)
            self.writer.flush(now)
            self.writer.snapshot(value)

    rclpy.init(args=args)
    node = None
    try:
        node = DiagnosticNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
