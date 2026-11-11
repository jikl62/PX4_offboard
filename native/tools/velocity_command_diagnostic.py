#!/usr/bin/env python3
# 只读速度诊断模块：目标速度是制导意图，不是飞机自身速度或 PX4 动力输出。
# 接收、时效、来源顺序、事件证据和写盘各司其职；全路径 control_authority 固定 false。
"""只读接收视觉目标速度：检查消息、保存诊断，不向飞控发送控制量。

阅读顺序：Receiver.receive() 处理新消息，snapshot() 判断缓存是否仍可用，
DiagnosticWriter 保存快照和日志，main() 只负责连接 ROS 订阅与这些纯计算对象。
阈值为 0 表示尚未配置；即使诊断通过，control_authority 也始终为 false。
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time


# 与视觉端约定的正式类型和默认话题；速度采用机体 FRD：前/右/下，单位 m/s。
MESSAGE_TYPE = 'jikl_fc01_system/msg/VelocityCommand'
DEFAULT_TOPIC = '/fc01/vision/velocity_command'
MAX_TEXT = 256
STREAM_HISTORY_CAPACITY = 32
LIMIT_NAMES = ('timeout_sec', 'max_speed_mps', 'max_source_age_sec', 'future_tolerance_sec')


def _finite(value):
    """只接受可表示为有限浮点数的数值；超大整数也按无效输入处理。"""
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


# 整数合同排除 bool，防止 true 被当成序号、发布者数量或时间。
def _integer(value, maximum):
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= maximum


# 递归复制容器并将非有限浮点改为 null，只服务诊断序列化，不修补消息有效性。
def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


# 文本截断仅限制日志体积；receive 仍对未经截断的原字段进行身份与长度验证。
def _text(value):
    return value[:MAX_TEXT] if isinstance(value, str) else None


@dataclass(frozen=True)
class DiagnosticConfig:
    """诊断策略；数值阈值与发送来源必须由接口双方明确配置。"""
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
    """保存三个不同事实：最新收到的消息、当前候选、最后一次异常。

    accepted 只表示通过当时已配置的检查，不表示允许飞机运动。
    正常新消息可以恢复候选，但不能覆盖此前的异常证据。
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
        # 每个 (source_id, reset_counter) 的可信序号与时间水位，防止旧包重放。
        self._watermarks = OrderedDict()
        self._sequence = 0
        self._stamp = 0
        self._last_valid = None  # 历史上最后一次接受的消息，过期后仍保留供查错。
        self._candidate = None   # 当前候选；无效、过期或来源异常时立即撤销。
        self._last_observation_steady = None
        self._last_observation_ros = None
        self._timeout_reported = False
        self._events = deque(maxlen=128)
        self._event_id = 0

    # 单调接收时钟负责本地间隔，ROS 纳秒负责测量年龄；两种水位都只前进，回退立即撤销候选。
    def _observe_clocks(self, now, ros_now_ns):
        errors = []
        if self._last_observation_steady is not None and now < self._last_observation_steady:
            errors.append('RECEIVE_CLOCK_REGRESSION')
        if (self._last_observation_ros is not None and ros_now_ns
                and ros_now_ns < self._last_observation_ros):
            errors.append('ROS_CLOCK_REGRESSION')
        # 保留已经观察到的最大时刻。回退不能降低基准，否则连续的旧时间
        # 会在第二帧就重新通过；时钟追上基准后，正常新消息仍可恢复候选。
        if self._last_observation_steady is None or now > self._last_observation_steady:
            self._last_observation_steady = now
        if ros_now_ns and (self._last_observation_ros is None
                           or ros_now_ns > self._last_observation_ros):
            self._last_observation_ros = ros_now_ns
        if errors:
            self._candidate = None
            self.counters['clock_regression'] += 1
            self._event('clock_regression', errors, now, ros_now_ns)
        return errors

    def _observe_receive_timeout(self, now, ros_now_ns):
        # 必须先检查旧消息，再接收新消息。否则执行器停顿后恰好收到正常包，
        # 就会覆盖旧接收时间，让断流在定时快照尚未运行时消失。
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

    def _observe_candidate_expiry(self, now, ros_now_ns):
        # 收到新包和查询快照都走这里。接收年龄与源测量年龄是两个条件：
        # 刚收到的包也可能早已过时，不能只看它何时到达本机。
        errors = []
        if self._candidate is None:
            return errors
        candidate = self._candidate
        age = now - candidate['received_at']
        # 源年龄 = ROS 当前时间 − 测量时间；本机接收得晚仍可能收到一条已经过期的指令。
        source_age = ((ros_now_ns - candidate['stamp_ns']) / 1e9
                      if ros_now_ns and candidate['stamp_ns'] else None)
        if age < 0:
            errors.append('RECEIVE_CLOCK_REGRESSION')
        elif self.config.timeout_sec and age >= self.config.timeout_sec:
            errors.append('RECEIVE_TIMEOUT')
        if source_age is None:
            errors.append('ROS_CLOCK_UNAVAILABLE')
        elif self.config.max_source_age_sec and source_age >= self.config.max_source_age_sec:
            errors.append('SOURCE_AGE_LIMIT_EXCEEDED')
        elif self.config.future_tolerance_sec and source_age < -self.config.future_tolerance_sec:
            errors.append('SOURCE_STAMP_IN_FUTURE')
        if errors:
            self._candidate = None
            self.counters['source_expired'] += 1
            self._event('cache_revoked', errors, now, ros_now_ns,
                        last_candidate_sequence=candidate['sequence'],
                        last_candidate_source_age_sec=source_age)
        return errors

    # 事件 ID 与有界队列记录顺序；异常单独留存，队列溢出计数，正常新包不能洗掉先前异常。
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
        # 调用方可自由整理返回值，不能因此改写内部保留的异常证据。
        return _json_safe(event)

    def drain_events(self):
        """取出独立事件副本；last_event / last_anomaly 继续保留原始证据。"""
        result = _json_safe(list(self._events))
        self._events.clear()
        return result

    @property
    def last_observed_ros_ns(self):
        return self._last_observation_ros or 0

    # 发布者数量必须恰好为一；图冲突撤销当前候选，并单独记录数量转换。
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
        """按“旧缓存→本帧字段→消息顺序→候选更新”的顺序处理一帧。"""
        if not _finite(received_at) or received_at < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('independent receive clocks must be finite and nonnegative')

        # 1. 先记录旧缓存的失效和来源变化，不能让后面的正常数据抹掉空窗。
        clock_errors = self._observe_clocks(received_at, ros_now_ns)
        self._observe_receive_timeout(received_at, ros_now_ns)
        self._observe_candidate_expiry(received_at, ros_now_ns)
        self.observe_publishers(publisher_count, received_at, ros_now_ns)
        self.counters['received'] += 1

        # 2. 原始字段用于检查；日志中的文本会截断，但不能用截断值替代检查。
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

        # 3. 检查来源、reset、序号与测量时间；不合格数据不得推进可信水位。
        self._check_stream_order(source, reset, sequence, stamp_ns, valid, errors,
                                 received_at, ros_now_ns)

        # 4. 只有本帧全部检查通过，才建立新的当前候选。
        # last_message 保存本次观测；last_anomaly 由 _event 单独保存。
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

    def _check_stream_order(self, source, reset, sequence, stamp_ns, valid, errors,
                           received_at, ros_now_ns):
        """追加顺序错误，并仅在本帧可信时更新防重放记录。"""
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
                # 历史满后拒绝新会话，不淘汰旧水位，否则旧包可能重新有效。
                # 已记录会话仍可继续接收，不需要重启飞机或清除飞行故障。
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
                        # reset 可重启序号，不能让同一发送源的测量时钟倒退。
                        errors.append('SOURCE_STAMP_NOT_ADVANCING_ACROSS_RESET')
                    self._sequence = sequence
                    if stamp_ns and stamp_ns <= self._stamp:
                        errors.append('STAMP_NOT_ADVANCING')
                        if stamp_ns < self._stamp:
                            errors.append('SOURCE_CLOCK_REGRESSION')
                            self.counters['clock_regression'] += 1
                    if stamp_ns:
                        self._stamp = max(self._stamp, stamp_ns)
                # 错误包不能污染可信序号或时间。
                # 唯一例外是其余字段全对的 valid=false：它明确撤销该条指令，
                # 也要消耗序号，防止之后用同一序号重放为 valid=true。
                advance = not errors or (errors == ['COMMAND_INVALID'] and valid is False)
                if advance and source_matches:
                    self._watermarks[identity] = (self._sequence, self._stamp)

    def _receive_freshness(self, age):
        """只判断接收年龄；不据此推断源测量仍然新鲜。"""
        if not self.config.timeout_sec:
            return 'UNCONFIGURED'
        if age is None:
            return 'NO_SAMPLE'
        if age < 0:
            return 'CLOCK_REGRESSION'
        if age >= self.config.timeout_sec:
            return 'EXPIRED'
        return 'WITHIN_RECEIVE_LIMIT'

    def _message_freshness(self, receive_freshness, last_message):
        """综合最新消息的错误、接收年龄和源年龄，供诊断窗口展示。"""
        if receive_freshness in ('EXPIRED', 'CLOCK_REGRESSION', 'NO_SAMPLE'):
            return receive_freshness
        if self.latest_errors:
            return 'LATEST_MESSAGE_INVALID'
        time_limits_configured = all((self.config.timeout_sec, self.config.max_source_age_sec,
                                      self.config.future_tolerance_sec))
        if not time_limits_configured or last_message is None:
            return 'UNCONFIGURED'
        source_age = last_message['source_age_sec']
        if source_age is None:
            return 'ROS_CLOCK_UNAVAILABLE'
        if source_age >= self.config.max_source_age_sec:
            return 'EXPIRED'
        if source_age < -self.config.future_tolerance_sec:
            return 'SOURCE_STAMP_IN_FUTURE'
        return 'CURRENT'

    def snapshot(self, now, ros_now_ns, publisher_count=None):
        """重新检查当前缓存，返回独立的诊断快照；历史数据不会自动续期。"""
        if not _finite(now) or now < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('snapshot clocks must be finite and nonnegative')
        self._observe_clocks(now, ros_now_ns)
        self._observe_receive_timeout(now, ros_now_ns)
        candidate_errors = self._observe_candidate_expiry(now, ros_now_ns)
        if publisher_count is not None:
            self.observe_publishers(publisher_count, now, ros_now_ns)
        age = now - self._last_valid['received_at'] if self._last_valid else None
        receive_freshness = self._receive_freshness(age)
        last = dict(self.last_message) if self.last_message is not None else None
        if last is not None:
            last['receive_age_sec'] = now - last['received_at']
            last['source_age_sec'] = ((ros_now_ns - last['stamp_ns']) / 1e9
                                      if ros_now_ns and last['stamp_ns'] else None)
        if self._candidate is None:
            candidate_errors.append('NO_CURRENT_CANDIDATE')
        if not all(getattr(self.config, name) > 0 for name in LIMIT_NAMES):
            candidate_errors.append('LIMITS_UNCONFIGURED')
        if not self.config.expected_source_id:
            candidate_errors.append('SOURCE_BINDING_UNCONFIGURED')
        if self.publisher_count != 1:
            candidate_errors.append('PUBLISHER_COUNT_NOT_ONE')
        freshness = self._message_freshness(receive_freshness, last)
        return _json_safe({
            'schema_version': 1,
            'kind': 'velocity_command_diagnostic',
            'message_type': MESSAGE_TYPE,
            'control_authority': False,
            'qualification': 'DIAGNOSTIC_ONLY',
            'limits': {name: getattr(self.config, name) or None for name in LIMIT_NAMES},
            'expected_source_id': self.config.expected_source_id or None,
            'stream_history_size': len(self._watermarks),
            'stream_history_capacity': STREAM_HISTORY_CAPACITY,
            'cache_scope': 'DIAGNOSTIC_ONLY',
            'cache_usable': not candidate_errors,
            'cache_errors': candidate_errors,
            'current_candidate': self._candidate if not candidate_errors else None,
            'historical_last_accepted_message': self._last_valid,
            'observed_at': now,
            'ros_now_ns': ros_now_ns,
            'publisher_count': self.publisher_count,
            'freshness': freshness,
            'receive_freshness': receive_freshness,
            'last_valid_age_sec': age,
            'last_valid_is_historical': True,
            'last_message': last,
            'latest_errors': list(self.latest_errors),
            'last_anomaly': self.last_anomaly,
            'last_event': self.last_event,
            'counters': dict(self.counters),
        })


class DiagnosticWriter:
    """写入完整快照，并限制日志频率、单文件大小与备份数量。"""
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
        """普通事件允许节流；异常先保存在内存，稍后合并写出。"""
        if self.directory is None:
            return False
        if self._pending_anomaly is not None:
            self.flush(now)
        if self.last_logged_at is not None and 0 <= now - self.last_logged_at < self.min_interval:
            self.suppressed += 1
            if event.get('reasons'):
                if self._pending_anomaly is None:
                    # 等待落盘期间也要隔离嵌套字段，调用方整理事件不能抹掉原因。
                    self._pending_anomaly = _json_safe(dict(event, coalesced_anomalies=1))
                else:
                    self._pending_anomaly['coalesced_anomalies'] += 1
                    reasons = self._pending_anomaly.setdefault('coalesced_reasons', [])
                    for reason in event['reasons']:
                        if reason not in reasons and len(reasons) < 64:
                            reasons.append(reason)
            return False
        return self._write(event, now)

    def flush(self, now, *, force=False):
        """写出待处理异常；正常退出用 force 跳过节流，写入成功后才清除。"""
        if self._pending_anomaly is None:
            return False
        if not force and self.last_logged_at is not None and 0 <= now - self.last_logged_at < self.min_interval:
            return False
        written = self._write(self._pending_anomaly, now)
        if written:
            self._pending_anomaly = None
        return written

    # 单条日志超上限时写明确截断事件，再按大小滚动；成功写入后才推进节流基准并清零抑制计数。
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
        # 先写临时文件再原子替换，避免读者拿到写了一半的 JSON。
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


def finalize_diagnostic(receiver, writer, topic, now, system_now_ns):
    """退出前保存最后异常和停止快照，不再查询可能已经关闭的 ROS 上下文。

    now 是本机单调时间，system_now_ns 是退出时系统时间；缓存检查继续使用
    最后观察到的 ROS 时间，不能把系统时间冒充模拟时钟或另一独立时钟。
    日志写入失败后仍尝试保存快照，最后再把第一个文件错误交给调用者。
    """
    if not _integer(system_now_ns, 2**64 - 1):
        raise ValueError('shutdown system time must be a nonnegative integer')
    ros_now_ns = receiver.last_observed_ros_ns
    value = receiver.snapshot(now, ros_now_ns)
    value.update(topic=topic, receiver_running=False,
                 stopped_at_monotonic=now, stopped_at_ros_ns=ros_now_ns,
                 stopped_at_system_ns=system_now_ns,
                 shutdown_clock_source='last_observed_ros_clock',
                 cache_usable=False, current_candidate=None)
    value['cache_errors'] = list(dict.fromkeys(value['cache_errors'] + ['RECEIVER_STOPPED']))
    failures = []
    for event in receiver.drain_events():
        try:
            writer.record(event, now)
        except (OSError, ValueError) as error:
            failures.append(error)
    try:
        writer.flush(now, force=True)
    except (OSError, ValueError) as error:
        failures.append(error)
    value['log_events_suppressed'] = writer.suppressed
    try:
        writer.snapshot(value)
    except (OSError, ValueError) as error:
        failures.append(error)
    if failures:
        raise failures[0]
    return value


# 只有执行入口才加载 ROS；创建订阅与诊断定时器，不创建飞控 publisher。核心接收器可独立离线测试。
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor
    from jikl_fc01_system.msg import VelocityCommand

    class DiagnosticNode(Node):
        def __init__(self):
            super().__init__('fc01_velocity_command_diagnostic')

            def parameter(name, default):
                # 参数仅在启动时读取；运行期明确拒绝修改，避免显示值与实际值不同。
                return self.declare_parameter(name, default, descriptor=ParameterDescriptor(
                    read_only=True, description='Startup-only diagnostic configuration; restart to change.')).value

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
            # ROS 接收与定时快照共用同一 Receiver；没有任何飞控 publisher。
            self.timer = self.create_timer(period, self.snapshot)
            self.get_logger().info('Diagnostic only; control_authority=false type=' + MESSAGE_TYPE +
                                   ' topic=' + self.topic + ' zero limits are UNCONFIGURED')
            self.snapshot()

        # 每次回调重新采集单调秒、ROS 纳秒及图中的发布者数量，不能将两种时钟混为一谈。
        def receive_context(self):
            return time.monotonic(), max(0, self.get_clock().now().nanoseconds), self.count_publishers(self.topic)

        def flush_events(self, now):
            for event in self.receiver.drain_events():
                self.writer.record(event, now)

        def receive(self, message):
            now, ros_now, publishers = self.receive_context()
            self.receiver.receive(message, now, ros_now, publishers)
            self.flush_events(now)

        # 没有新消息时也重新检查过期，落盘前排空事件并尝试写出待节流异常，缓存不会靠显示自动续期。
        def snapshot(self):
            now, ros_now, publishers = self.receive_context()
            value = self.receiver.snapshot(now, ros_now, publishers)
            value['topic'] = self.topic
            value['receiver_running'] = True
            value['log_events_suppressed'] = self.writer.suppressed
            self.flush_events(now)
            self.writer.flush(now)
            self.writer.snapshot(value)

    rclpy.init(args=args)
    node = None
    try:
        node = DiagnosticNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                finalize_diagnostic(node.receiver, node.writer, node.topic,
                                    time.monotonic(), time.time_ns())
            except Exception as error:
                # 此时 ROS logger 可能已经关闭；写盘失败也不能掩盖原运行异常。
                print('FC01_DIAGNOSTIC_FINALIZE_FAILED ' + repr(error), file=sys.stderr)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
