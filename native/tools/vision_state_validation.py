#!/usr/bin/env python3
"""Pure, diagnostic-only validation around the existing Odometry conversion.

No ROS imports, publishers, controller calls, clock estimation, or covariance
frame transforms live here. An accepted sample is an offline data observation,
never permission to control an aircraft.
"""
from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass, field
import importlib.util
import math
from pathlib import Path
import sys
from typing import Optional


def _load_adapter():
    # Source-tree and installed sibling layouts both keep the actual converter.
    name = '_fc01_state_validation_odometry_adapter'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            name, Path(__file__).with_name('vision_odometry_adapter.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


adapter = _load_adapter()
AdapterConfig = adapter.AdapterConfig
ConvertedState = adapter.ConvertedState


def _number(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def _integer(value, maximum):
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= maximum


def _stamp(value):
    if value is None:
        return None
    sec, ns = getattr(value, 'sec', None), getattr(value, 'nanosec', None)
    if not _integer(sec, 2**31-1) or not _integer(ns, 999999999):
        return None
    return sec*1000000000+ns or None


@dataclass(frozen=True)
class ClockPolicy:
    # local ROS time = source measurement time + offset_ns, if confirmed.
    offset_ns: Optional[int] = None
    confirmed: bool = False

    def __post_init__(self):
        if type(self.confirmed) is not bool:
            raise ValueError('clock confirmed must be bool')
        if self.offset_ns is not None and (not isinstance(self.offset_ns, int) or isinstance(self.offset_ns, bool)):
            raise ValueError('clock offset_ns must be an integer or None')
        if self.confirmed and self.offset_ns is None:
            raise ValueError('confirmed clock requires an explicit offset, including zero')


@dataclass(frozen=True)
class ValidationConfig:
    adapter: AdapterConfig
    clock: ClockPolicy = field(default_factory=ClockPolicy)
    receive_timeout_sec: Optional[float] = None
    max_source_age_sec: Optional[float] = None
    future_tolerance_sec: Optional[float] = None
    min_quality: Optional[float] = None
    nominal_quality_min: Optional[float] = None
    max_position_step_m: Optional[float] = None
    max_velocity_step_mps: Optional[float] = None
    max_attitude_step_rad: Optional[float] = None
    # Upper bounds are raw Odometry variances, in their declared raw frames.
    max_pose_variance: Optional[tuple] = None
    max_twist_variance: Optional[tuple] = None
    # Resource bound, not a flight tuning value. Never evict retired epochs.
    epoch_history_capacity: int = 128

    def __post_init__(self):
        self.adapter.validate()
        for name in ('source_id', 'input_frame_id', 'input_body_frame_id', 'extrinsics_id', 'alignment_id'):
            if len(getattr(self.adapter, name)) > 256:
                raise ValueError('adapter identity strings must be at most 256 characters')
        if not isinstance(self.clock, ClockPolicy):
            raise ValueError('explicit ClockPolicy required')
        if not _integer(self.epoch_history_capacity, 4096) or self.epoch_history_capacity < 1:
            raise ValueError('epoch_history_capacity must be an integer in [1,4096]')
        for name in ('receive_timeout_sec', 'max_source_age_sec', 'future_tolerance_sec',
                     'max_position_step_m', 'max_velocity_step_mps', 'max_attitude_step_rad'):
            value = getattr(self, name)
            if value is not None and (not _number(value) or value < 0 or
                                      (value == 0 and name != 'future_tolerance_sec')):
                raise ValueError(name+' must be finite and positive (future tolerance may be zero), or None')
        for name in ('min_quality', 'nominal_quality_min'):
            value = getattr(self, name)
            if value is not None and (not _number(value) or not 0 <= value <= 1):
                raise ValueError(name+' must be in [0,1] or None')
        if (self.min_quality is not None and self.nominal_quality_min is not None
                and self.nominal_quality_min < self.min_quality):
            raise ValueError('nominal quality must not be below minimum quality')
        for name in ('max_pose_variance', 'max_twist_variance'):
            value = getattr(self, name)
            if value is not None:
                if not isinstance(value, (tuple, list)) or len(value) != 6 or not all(_number(x) and x >= 0 for x in value):
                    raise ValueError(name+' must contain six finite nonnegative raw variances')
                object.__setattr__(self, name, tuple(value))


@dataclass(frozen=True)
class SampleMetadata:
    # These are explicit local adapter declarations, not new ROS message fields.
    pose_covariance_valid: bool = False
    twist_covariance_valid: bool = False
    relocalization_id: str = ''

    def __post_init__(self):
        if type(self.pose_covariance_valid) is not bool or type(self.twist_covariance_valid) is not bool:
            raise ValueError('covariance declarations must be bool')
        if not isinstance(self.relocalization_id, str) or len(self.relocalization_id) > 256:
            raise ValueError('relocalization_id must be an explicit string of at most 256 characters')


def covariance_diagnostic(values, declared, upper=None):
    """Check the raw 6x6 matrix; do not assign transformed covariance validity.

    The scaled tolerance below handles numerical roundoff in symmetry/PSD. It
    is a matrix arithmetic tolerance, not a flight uncertainty threshold.
    """
    if not declared:
        return dict(status='UNKNOWN', reasons=[], variances=None, transformed_valid=False)
    try:
        entries = tuple(values)
    except TypeError:
        entries = ()
    if len(entries) != 36 or not all(_number(x) for x in entries):
        return dict(status='INVALID', reasons=['COVARIANCE_NONFINITE_OR_WRONG_SIZE'], variances=None)
    diagonal = tuple(entries[i*6+i] for i in range(6))
    reasons = []
    if any(x < 0 for x in diagonal):
        reasons.append('COVARIANCE_NEGATIVE_VARIANCE')
    scale = max((abs(x) for x in entries), default=0.)
    # Normalize each axis by its own standard deviation. A single very large
    # variance must not hide an indefinite block in other physical units.
    residual = [[0.]*6 for _ in range(6)]
    tolerance = 1.e-10
    if not reasons:
        roots = tuple(math.sqrt(x) for x in diagonal)
        for i in range(6):
            for j in range(6):
                value = entries[i*6+j]
                if roots[i] == 0 or roots[j] == 0:
                    if value != 0:
                        reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                    continue
                normalized = (value/max(roots[i], roots[j]))/min(roots[i], roots[j])
                if not math.isfinite(normalized) or abs(normalized) > 1.+tolerance:
                    reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                residual[i][j] = normalized
        if any(abs(residual[i][j]-residual[j][i]) > tolerance for i in range(6) for j in range(i)):
            reasons.append('COVARIANCE_NOT_SYMMETRIC')
    if not reasons:
        # Pivoted semidefinite Cholesky on a copy. A zero pivot may have no
        # residual cross terms; merely testing the six diagonals is insufficient.
        for column in range(6):
            pivot = max(range(column, 6), key=lambda i: residual[i][i])
            residual[column], residual[pivot] = residual[pivot], residual[column]
            for row in residual:
                row[column], row[pivot] = row[pivot], row[column]
            value = residual[column][column]
            if value < -tolerance:
                reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                break
            if value <= tolerance:
                if any(abs(residual[i][j]) > tolerance for i in range(column, 6) for j in range(column, 6)):
                    reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                break
            for i in range(column+1, 6):
                for j in range(i, 6):
                    updated = residual[i][j]-(residual[i][column]/value)*residual[j][column]
                    residual[i][j] = residual[j][i] = updated
                    if not math.isfinite(updated):
                        reasons.append('COVARIANCE_DECOMPOSITION_NONFINITE')
                        break
                if reasons:
                    break
            if reasons:
                break
    if upper is not None and any(value > limit for value, limit in zip(diagonal, upper)):
        reasons.append('COVARIANCE_UNCERTAINTY_LIMIT_EXCEEDED')
    status = 'INVALID' if reasons else ('DECLARED_ZERO' if not scale else 'VALID_RAW')
    return dict(status=status, reasons=list(dict.fromkeys(reasons)), variances=diagonal,
                transformed_valid=False)


@dataclass(frozen=True)
class ValidationResult:
    state: ConvertedState
    accepted: bool
    diagnostics: dict
    control_authority: bool = field(default=False, init=False)


class Validator:
    """A bounded receiver for exact Odometry/Health pairs, with recoverable cache.

    Unknown identity/frame changes remain rejected by the configured converter.
    Epoch diagnostics never automatically infer a new coordinate transform.
    """
    def __init__(self, config):
        if not isinstance(config, ValidationConfig):
            raise ValueError('ValidationConfig required')
        self.config = config
        self.counters = Counter()
        self.reason_counts = Counter()
        self.last_anomaly = None
        self._events = deque(maxlen=128)
        self._event_id = 0
        self._epoch = None
        self._accepted_epoch = None
        self._trusted_epochs = set()
        self._sequence = 0
        # A pose/reset epoch may restart a producer sequence, never the
        # confirmed measurement clock. Only accepted samples advance this.
        self._source_stamp = 0
        self._last_receive = self._last_ros = None
        self._baseline = self._cache = None
        self._last_reasons = ('NO_SAMPLE',)
        self._previously_rejected = False

    def _event(self, kind, reasons, **fields):
        self._event_id += 1
        value = dict(event_id=self._event_id, kind=kind, reasons=list(reasons),
                     control_authority=False, **fields)
        if len(self._events) == self._events.maxlen:
            self.counters['events_dropped'] += 1
        self._events.append(value)
        self.counters[kind] += 1
        self.reason_counts.update(reasons)
        if kind in ('rejected', 'expired', 'clock_regression'):
            self.last_anomaly = deepcopy(value)
        return value

    def drain_events(self):
        values = deepcopy(list(self._events))
        self._events.clear()
        return values

    def _observe_clocks(self, now, ros_now_ns):
        reasons = []
        if self._last_receive is not None and now < self._last_receive:
            reasons.append('RECEIVE_CLOCK_REGRESSION')
        if self._last_ros is not None and ros_now_ns < self._last_ros:
            reasons.append('ROS_CLOCK_REGRESSION')
        self._last_receive = now if self._last_receive is None else max(now, self._last_receive)
        self._last_ros = ros_now_ns if self._last_ros is None else max(ros_now_ns, self._last_ros)
        return reasons

    def _cache_age_errors(self, now, ros_now_ns):
        if self._cache is None:
            return [], None
        state, received, _, _ = self._cache
        source_age = (ros_now_ns-state.stamp_ns)/1e9 if ros_now_ns else None
        reasons = []
        if now-received >= self.config.receive_timeout_sec:
            reasons.append('RECEIVE_SAMPLE_EXPIRED')
        if source_age is None:
            reasons.append('SOURCE_CLOCK_UNAVAILABLE')
        elif source_age >= self.config.max_source_age_sec or source_age < -self.config.future_tolerance_sec:
            reasons.append('SOURCE_SAMPLE_OUTSIDE_TIME_WINDOW')
        return reasons, source_age

    def _unknowns(self):
        names = ('receive_timeout_sec', 'max_source_age_sec', 'future_tolerance_sec',
                 'min_quality', 'nominal_quality_min', 'max_position_step_m',
                 'max_velocity_step_mps', 'max_attitude_step_rad',
                 'max_pose_variance', 'max_twist_variance')
        unknown = [name for name in names if getattr(self.config, name) is None]
        if not self.config.clock.confirmed:
            unknown.append('clock_offset')
        return unknown

    def _result(self, state, accepted, reasons, tracking='UNKNOWN', covariance=None, **fields):
        diagnostics = dict(control_authority=False, qualification='DIAGNOSTIC_ONLY',
                           reasons=list(reasons), unknown=self._unknowns(),
                           tracking_status=tracking, covariance=deepcopy(covariance) if covariance is not None else {},
                           counters=dict(self.counters), reason_counts=dict(self.reason_counts),
                           trusted_epoch_count=len(self._trusted_epochs),
                           epoch_history_capacity=self.config.epoch_history_capacity,
                           last_anomaly=deepcopy(self.last_anomaly), event_id=self._event_id, **fields)
        return ValidationResult(deepcopy(state), accepted, diagnostics)

    def _invalid(self, reasons, **fields):
        return self._result(ConvertedState(source_id=self.config.adapter.source_id,
                                          reason=reasons[0] if reasons else 'INVALID'),
                            False, reasons, **fields)

    def _observe_epoch(self, raw, health, metadata):
        if health is None:
            return None
        source, reset = getattr(health, 'source_id', None), getattr(health, 'reset_counter', None)
        frame = getattr(getattr(raw, 'header', None), 'frame_id', None)
        body = getattr(raw, 'child_frame_id', None)
        if not all(isinstance(x, str) and x and len(x) <= 256 for x in (source, frame, body)) or not _integer(reset, 2**32-1):
            return None
        epoch = (source, frame, body, reset, metadata.relocalization_id)
        if self._epoch is not None and epoch != self._epoch:
            reasons = []
            if source != self._epoch[0]:
                reasons.append('SOURCE_EPOCH_CHANGED')
            if (frame, body) != self._epoch[1:3]:
                reasons.append('FRAME_EPOCH_CHANGED')
            if reset != self._epoch[3]:
                reasons.append('ESTIMATOR_RESET')
            if metadata.relocalization_id != self._epoch[4]:
                reasons.append('RELOCALIZATION')
            self._event('epoch_change', reasons, previous_epoch=self._epoch, epoch=epoch)
        self._epoch = epoch
        return epoch

    def receive(self, raw, health, received_at, ros_now_ns, metadata=None):
        metadata = metadata or SampleMetadata()
        if not isinstance(metadata, SampleMetadata):
            raise ValueError('SampleMetadata required')
        if not _number(received_at) or received_at < 0 or not _integer(ros_now_ns, 2**64-1):
            raise ValueError('independent receive clocks must be finite and nonnegative')
        self.counters['received'] += 1
        epoch = self._observe_epoch(raw, health, metadata)
        same_epoch = epoch is not None and epoch == self._accepted_epoch
        reasons = self._observe_clocks(received_at, ros_now_ns)
        if epoch is None:
            reasons.append('EPOCH_IDENTITY_INVALID')
        elif not same_epoch:
            if epoch in self._trusted_epochs:
                reasons.append('RETIRED_EPOCH_REPLAY')
            elif len(self._trusted_epochs) >= self.config.epoch_history_capacity:
                reasons.append('EPOCH_HISTORY_CAPACITY_REACHED')
        if not reasons:
            expired, _ = self._cache_age_errors(received_at, ros_now_ns)
            if expired:
                # A receive callback may run before a periodic current() check.
                # New traffic cannot erase the already-proven old-cache gap.
                self._cache = None
                self._previously_rejected = True
                self._event('expired', expired, observed_at=received_at, observed_on_receive=True)
        raw_stamp = _stamp(getattr(getattr(raw, 'header', None), 'stamp', None))
        health_stamp = _stamp(getattr(health, 'stamp', None))
        seq = getattr(health, 'sequence', None)
        if type(getattr(health, 'schema_version', None)) is not int or health.schema_version != 1:
            reasons.append('HEALTH_VERSION_INVALID')
        if not _integer(getattr(health, 'reset_counter', None), 2**32-1):
            reasons.append('RESET_COUNTER_INVALID')
        if not _number(getattr(health, 'quality', None)):
            reasons.append('QUALITY_NOT_FINITE_NUMBER')
        if health is not None:
            flags = ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                     'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                     'metric_scale_valid', 'quality_valid', 'position_valid',
                     'velocity_valid', 'attitude_valid', 'angular_velocity_valid')
            if any(hasattr(health, name) and type(getattr(health, name)) is not bool for name in flags):
                reasons.append('HEALTH_CONFIRMATION_NOT_BOOL')
        if not _integer(seq, 2**64-1) or not seq:
            reasons.append('SEQUENCE_INVALID')
        elif same_epoch:
            if seq == self._sequence:
                reasons.append('DUPLICATE_SEQUENCE')
            elif seq < self._sequence:
                reasons.append('SEQUENCE_REGRESSION')
        if raw_stamp is None or health_stamp is None or raw_stamp != health_stamp:
            reasons.append('INVALID_OR_UNPAIRED_STAMP')
        elif raw_stamp <= self._source_stamp:
            reasons.append('MEASUREMENT_CLOCK_NOT_ADVANCING')
        clock = self.config.clock
        if not clock.confirmed:
            reasons.append('CLOCK_OFFSET_UNCONFIRMED')
        if any(getattr(self.config, key) is None for key in
               ('receive_timeout_sec', 'max_source_age_sec', 'future_tolerance_sec')):
            reasons.append('TIMING_POLICY_UNCONFIGURED')
        mapped_stamp = raw_stamp + clock.offset_ns if raw_stamp is not None and clock.confirmed else None
        if mapped_stamp is not None and not 0 < mapped_stamp <= (2**31)*1000000000-1:
            reasons.append('MAPPED_STAMP_OUT_OF_RANGE')
            mapped_stamp = None
        source_age = (ros_now_ns-mapped_stamp)/1e9 if ros_now_ns and mapped_stamp else None
        if source_age is not None:
            if self.config.max_source_age_sec is not None and source_age >= self.config.max_source_age_sec:
                reasons.append('SOURCE_SAMPLE_EXPIRED')
            if self.config.future_tolerance_sec is not None and source_age < -self.config.future_tolerance_sec:
                reasons.append('SOURCE_SAMPLE_IN_FUTURE')
        elif clock.confirmed:
            reasons.append('SOURCE_CLOCK_UNAVAILABLE')

        covariance = {}
        for name, declared, upper in (
            ('pose', metadata.pose_covariance_valid, self.config.max_pose_variance),
            ('twist', metadata.twist_covariance_valid, self.config.max_twist_variance)):
            values = getattr(getattr(raw, name, None), 'covariance', None)
            value = covariance_diagnostic(values, declared, upper)
            covariance[name] = value
            reasons.extend(name.upper()+'_'+x for x in value['reasons'])
            if not declared and upper is not None:
                reasons.append(name.upper()+'_COVARIANCE_UNKNOWN_WITH_LIMIT')

        converted = ConvertedState(source_id=self.config.adapter.source_id)
        if raw_stamp is not None and health_stamp == raw_stamp:
            try:
                source_raw, source_health = raw, health
                if mapped_stamp is not None:
                    source_raw, source_health = deepcopy(raw), deepcopy(health)
                    for stamp in (source_raw.header.stamp, source_health.stamp):
                        stamp.sec, stamp.nanosec = divmod(mapped_stamp, 1000000000)
                converted = adapter.convert_odometry(source_raw, source_health, self.config.adapter)
            except (AttributeError, TypeError, ValueError, OverflowError) as error:
                reasons.append('MALFORMED_ODOMETRY_OR_HEALTH')
                converted.reason = type(error).__name__
        if converted.valid_fields & adapter.REQUIRED != adapter.REQUIRED:
            reasons.append(converted.reason)
        tracking = 'UNKNOWN'
        if getattr(health, 'tracking', None) is False:
            tracking = 'LOST'
        elif converted.valid_fields & adapter.QUALITY:
            threshold = self.config.nominal_quality_min
            tracking = ('TRACKING_UNCLASSIFIED' if threshold is None else
                        ('NORMAL' if converted.quality >= threshold else 'DEGRADED'))
            if self.config.min_quality is not None and converted.quality < self.config.min_quality:
                reasons.append('QUALITY_BELOW_EXPLICIT_MINIMUM')
        if same_epoch and self._baseline is not None and converted.valid_fields & adapter.REQUIRED == adapter.REQUIRED:
            for name, value, limit in (
                ('POSITION', math.dist(converted.position, self._baseline.position), self.config.max_position_step_m),
                ('VELOCITY', math.dist(converted.velocity, self._baseline.velocity), self.config.max_velocity_step_mps),
                ('ATTITUDE', 2*math.acos(min(1., abs(sum(a*b for a, b in zip(converted.attitude, self._baseline.attitude))))),
                 self.config.max_attitude_step_rad)):
                if limit is not None and value > limit:
                    reasons.append(name+'_CONTINUITY_LIMIT_EXCEEDED')
        reasons = list(dict.fromkeys(reasons))
        if reasons:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(reasons)
            self._event('rejected', reasons, received_at=received_at, raw_stamp_ns=raw_stamp)
            return self._invalid(reasons, tracking=tracking, covariance=covariance,
                                 source_age_sec=source_age, raw_stamp_ns=raw_stamp,
                                 converted_valid_fields=converted.valid_fields, freshness='INVALID')
        if self._previously_rejected:
            self._event('recovered', [], received_at=received_at)
        self._previously_rejected = False
        self._accepted_epoch = epoch
        self._trusted_epochs.add(epoch)
        self._sequence, self._source_stamp = seq, raw_stamp
        self._baseline = deepcopy(converted)
        self._cache = (deepcopy(converted), received_at, tracking, deepcopy(covariance))
        self._last_reasons = ()
        self._event('accepted', [], received_at=received_at, raw_stamp_ns=raw_stamp)
        return self._result(converted, True, [], tracking, covariance,
                            source_age_sec=source_age, raw_stamp_ns=raw_stamp, freshness='CURRENT')

    def current(self, now, ros_now_ns):
        if not _number(now) or now < 0 or not _integer(ros_now_ns, 2**64-1):
            raise ValueError('independent current clocks must be finite and nonnegative')
        clock_errors = self._observe_clocks(now, ros_now_ns)
        if clock_errors:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(clock_errors)
            self._event('clock_regression', clock_errors, observed_at=now)
            return self._invalid(clock_errors, freshness='INVALID')
        if self._cache is None:
            return self._invalid(self._last_reasons or ('NO_CURRENT_SAMPLE',), freshness='NO_CURRENT_SAMPLE')
        state, received, tracking, covariance = self._cache
        reasons, source_age = self._cache_age_errors(now, ros_now_ns)
        if reasons:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(reasons)
            self._event('expired', reasons, observed_at=now)
            return self._invalid(reasons, freshness='INVALID', source_age_sec=source_age)
        return self._result(state, True, [], tracking, covariance,
                            receive_age_sec=now-received, source_age_sec=source_age, freshness='CURRENT')
