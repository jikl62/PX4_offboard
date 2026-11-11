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
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time


def _load_resource_budget():
    # 从同目录加载唯一预算实现，源码与安装后均不依赖当前工作目录或 PYTHONPATH。
    #
    # 无参数，返回已缓存的模块对象；首次调用会登记 sys.modules 并执行该文件。
    # 文件读取/模块执行错误原样传播；这里不导入 ROS，不创建 Context 或通信端点。
    """从同目录加载唯一预算实现，源码与安装后导入均不依赖工作目录或 PYTHONPATH。"""
    name = '_fc01_vision_resource_budget'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            name, Path(__file__).with_name('vision_resource_budget.py')
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


resource_budget = _load_resource_budget()

# 与视觉端约定的正式类型和默认话题；速度采用机体 FRD：前/右/下，单位 m/s。
MESSAGE_TYPE = 'jikl_fc01_system/msg/VelocityCommand'
DEFAULT_TOPIC = '/fc01/vision/velocity_command'
MAX_TEXT = resource_budget.IDENTITY_MAX_BYTES
STREAM_HISTORY_CAPACITY = 32
LIMIT_NAMES = ('timeout_sec', 'max_speed_mps', 'max_source_age_sec', 'future_tolerance_sec')


def _finite(value):
    # 判断 value 是否为有限的 Python 数值并返回 bool，不修改输入。
    #
    # bool 虽是 int 子类，但在协议中不算数值；超大整数导致转换溢出时返回 False。
    # 这是拒绝前的检查，不能把 NaN/Inf 替换成零后继续接收。
    """只接受可表示为有限浮点数的数值；超大整数也按无效输入处理。"""
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _integer(value, maximum):
    # 返回 value 是否为 [0, maximum] 内的真正 int；排除 bool，不做隐式类型转换。
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= maximum


def _json_safe(value):
    # 为诊断复制 dict/list/tuple，把非有限 float 转为 JSON 的 null。
    #
    # 返回独立的嵌套容器，原输入不改；其他标量原样保留，tuple 在输出中变为 list。
    # 仅服务显示和文件证据，不能用这个“整理后的值”代替接收时对原消息的有效性检查。
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _text(value):
    # 返回按统一 UTF-8 字节预算截断的诊断文本；原身份仍由 receive 独立验证。
    return resource_budget.bounded_text(value)


@dataclass(frozen=True)
class DiagnosticConfig:
    """诊断策略；数值阈值与发送来源必须由接口双方明确配置。"""

    timeout_sec: float = 0.0
    max_speed_mps: float = 0.0
    max_source_age_sec: float = 0.0
    future_tolerance_sec: float = 0.0
    expected_source_id: str = ''

    def __post_init__(self):
        # 构造后验证四项非负有限阈值和可选来源身份，不转换或猜测参数。
        #
        # 阈值 0 表示未配置，不等于“已证明不限时且可用于导航”；来源空串表示未绑定。
        # 不合法时抛 ValueError，首个失败项按 LIMIT_NAMES 的固定顺序报告。
        for name in LIMIT_NAMES:
            value = getattr(self, name)
            if not _finite(value) or value < 0:
                raise ValueError(name + ' must be finite and nonnegative; 0 means unconfigured')
        if not resource_budget.valid_identity(self.expected_source_id, allow_empty=True):
            raise ValueError(
                'expected_source_id must be empty (unbound) or an explicit source identifier'
            )


class Receiver:
    """保存三个不同事实：最新收到的消息、当前候选、最后一次异常。

    accepted 只表示通过当时已配置的检查，不表示允许飞机运动。
    正常新消息可以恢复候选，但不能覆盖此前的异常证据。
    """

    def __init__(self, config=None):
        # 以可选 DiagnosticConfig 建立本接收器独有的观察、候选、可信水位和有界事件队列。
        #
        # 不读取时间、不导入 ROS、不写盘；各时刻由 receive/snapshot 的调用者明确传入。
        # last_message 是最新观察，_last_valid 是最后接受的历史，_candidate 才是当前诊断候选。
        self.config = config or DiagnosticConfig()
        self.counters = dict.fromkeys(
            (
                'received',
                'accepted',
                'invalid',
                'duplicate',
                'out_of_order',
                'source_change',
                'reset_change',
                'source_count',
                'timeout',
                'clock_regression',
                'source_expired',
                'anomalies',
                'events_dropped',
            ),
            0,
        )
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
        self._candidate = None  # 当前候选；无效、过期或来源异常时立即撤销。
        self._last_observation_steady = None
        self._last_observation_ros = None
        self._timeout_reported = False
        self._events = deque(maxlen=128)
        self._event_id = 0

    def _observe_clocks(self, now, ros_now_ns):
        # 检查单调秒 now 和 ROS 纳秒 ros_now_ns 是否回退，返回本轮错误列表。
        #
        # 保留各自最大观察水位；回退会撤销候选、计数并留事件，不降低旧基准。
        # ros_now_ns=0 表示当前不可用；它不替代源年龄检查，也不让系统时钟冒充 ROS 时钟。
        errors = []
        if self._last_observation_steady is not None and now < self._last_observation_steady:
            errors.append('RECEIVE_CLOCK_REGRESSION')
        if (
            self._last_observation_ros is not None
            and ros_now_ns
            and ros_now_ns < self._last_observation_ros
        ):
            errors.append('ROS_CLOCK_REGRESSION')
        # 保留已经观察到的最大时刻。回退不能降低基准，否则连续的旧时间
        # 会在第二帧就重新通过；时钟追上基准后，正常新消息仍可恢复候选。
        if self._last_observation_steady is None or now > self._last_observation_steady:
            self._last_observation_steady = now
        if ros_now_ns and (
            self._last_observation_ros is None or ros_now_ns > self._last_observation_ros
        ):
            self._last_observation_ros = ros_now_ns
        if errors:
            self._candidate = None
            self.counters['clock_regression'] += 1
            self._event('clock_regression', errors, now, ros_now_ns)
        return errors

    def _observe_receive_timeout(self, now, ros_now_ns):
        # 先检查最后接受包的本机接收年龄；无返回值，过期撤销候选并只记一次超时。
        #
        # now 是单调秒；timeout_sec=0 时不判断这一项。必须在接收新包之前调用，
        # 否则新包可能覆盖旧 received_at，使中间断流没有证据。
        if (
            self._last_valid is None
            or not self.config.timeout_sec
            or now - self._last_valid['received_at'] < self.config.timeout_sec
        ):
            return
        self._candidate = None
        if not self._timeout_reported:
            self._timeout_reported = True
            self.counters['timeout'] += 1
            self._event(
                'timeout',
                ['RECEIVE_TIMEOUT'],
                now,
                ros_now_ns,
                last_valid_age_sec=now - self._last_valid['received_at'],
                last_valid_sequence=self._last_valid['sequence'],
            )

    def _observe_candidate_expiry(self, now, ros_now_ns):
        # 检查当前候选的接收年龄和源测量年龄，返回本次发现的撤销原因。
        #
        # 接收年龄用单调秒，源年龄用 ROS 纳秒差除以 1e9，二者不能相互替代。
        # 失败时清当前候选并记一次 cache_revoked；历史接受包与既有异常仍保留。
        errors = []
        if self._candidate is None:
            return errors
        candidate = self._candidate
        age = now - candidate['received_at']
        # 源年龄 = ROS 当前时间 − 测量时间；本机接收得晚仍可能收到一条已经过期的指令。
        source_age = (
            (ros_now_ns - candidate['stamp_ns']) / 1e9
            if ros_now_ns and candidate['stamp_ns']
            else None
        )
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
            self._event(
                'cache_revoked',
                errors,
                now,
                ros_now_ns,
                last_candidate_sequence=candidate['sequence'],
                last_candidate_source_age_sec=source_age,
            )
        return errors

    def _event(self, kind, reasons, now, ros_now_ns, **fields):
        # 为事件分配递增 ID、更新统计/历史并入有界队列，返回独立的 JSON 安全副本。
        #
        # kind/reasons/fields 只描述诊断事实；control_authority 始终为 False。
        # 队列满时淘汰最旧项并计数；last_anomaly 独立保存最近异常，不被普通健康事件清除。
        self._event_id += 1
        event = _json_safe(
            dict(
                event_id=self._event_id,
                kind=kind,
                reasons=list(reasons),
                received_at=now,
                ros_now_ns=ros_now_ns,
                control_authority=False,
                **fields,
            )
        )
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
        # 返回独立事件副本并清空待消费队列；last_event/last_anomaly 继续保留证据。
        #
        # 这是一次有副作用的“取走”，不是只读查询；调用者写盘失败不会自动把事件放回队列。
        """取出独立事件副本；last_event / last_anomaly 继续保留原始证据。"""
        result = _json_safe(list(self._events))
        self._events.clear()
        return result

    @property
    def last_observed_ros_ns(self):
        # 只读返回见过的最大 ROS 纳秒；尚无非零观察时返回 0，不调用系统时钟。
        return self._last_observation_ros or 0

    def observe_publishers(self, count, now, ros_now_ns):
        # 检查发布者数量 count，返回数量变化事件；数量未变返回 None。
        #
        # count 必须为非负 int，非法值抛 ValueError；不为一时撤销当前候选并留证。
        # 数量恢复为一只表示图恢复，不能凭此复活旧候选；还要新消息重新通过各项检查。
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
        return self._event(
            'publisher_count',
            reasons,
            now,
            ros_now_ns,
            previous_count=previous,
            publisher_count=count,
        )

    def receive(self, message, received_at, ros_now_ns, publisher_count):
        # 按“旧缓存→本帧字段→消息顺序→候选更新”处理消息，返回该帧诊断事件副本。
        #
        # message 是 VelocityCommand 或同字段替身；received_at 为本机单调秒，
        # ros_now_ns 为 ROS 当前纳秒，publisher_count 为此时图中数量。这里不自行查询时间或 ROS。
        # 时钟参数/发布者数量不合基本类型合同时抛 ValueError；普通字段值校验失败记入事件。
        # 这不保证接收永不抛错：属性 getter、内存分配、序列化等异常仍向外传播。
        # 接受只更新诊断候选，不产生飞控输出，也不取得控制权；坏包立即撤销候选但保留历史。
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
        sample = dict(
            schema_version=schema,
            sequence=sequence,
            stamp=dict(sec=sec, nanosec=nanosec),
            stamp_ns=stamp_ns or None,
            reset_counter=reset,
            source_id=_text(source),
            frame_id=_text(frame),
            valid=valid,
            vx=velocities[0],
            vy=velocities[1],
            vz=velocities[2],
            received_at=received_at,
            source_age_at_receive_sec=source_age,
            publisher_count=publisher_count,
        )
        errors = list(clock_errors)
        if schema != 1 or isinstance(schema, bool):
            errors.append('SCHEMA_VERSION_INVALID')
        if not _integer(sequence, 2**64 - 1) or not sequence:
            errors.append('SEQUENCE_INVALID')
        if not stamp_ok or not stamp_ns:
            errors.append('STAMP_INVALID')
        if not _integer(reset, 2**32 - 1):
            errors.append('RESET_COUNTER_INVALID')
        if not resource_budget.valid_identity(source):
            errors.append('SOURCE_ID_INVALID')
        elif self.config.expected_source_id and source != self.config.expected_source_id:
            errors.append('SOURCE_ID_NOT_EXPECTED')
        if frame != 'body_frd':
            errors.append('FRAME_ID_MUST_BE_EXPLICIT_BODY_FRD')
        if not resource_budget.identity_within_budget(frame):
            errors.append('FRAME_ID_RESOURCE_LIMIT')
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
        self._check_stream_order(
            source, reset, sequence, stamp_ns, valid, errors, received_at, ros_now_ns
        )

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
        return self._event(
            'message', errors, received_at, ros_now_ns, accepted=accepted, sample=sample
        )

    def _check_stream_order(
        self, source, reset, sequence, stamp_ns, valid, errors, received_at, ros_now_ns
    ):
        # 追加来源/reset/序号/测量时间错误，仅在可信输入上提交防重放水位；无返回值。
        #
        # errors 是调用者本帧错误列表，函数会原位追加；同时可能更新观察身份、计数及流变化事件。
        # _identity/_sequence/_stamp 含本帧观察/试算，_watermarks 才是已确认的防重放历史。
        # 唯一允许带 COMMAND_INVALID 提交水位的情况是其他字段全对且 valid 恰为 False：
        # 显式撤销也要消耗序号，不能同序号改成 True 重放。历史满时拒绝新身份而不逐出旧证据。
        """追加顺序错误，并仅在本帧可信时更新防重放记录。"""
        identity_valid = _integer(reset, 2**32 - 1) and resource_budget.valid_identity(source)
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
                self._event(
                    'stream_change',
                    changes,
                    received_at,
                    ros_now_ns,
                    previous_source_id=self._identity[0],
                    previous_reset_counter=self._identity[1],
                    source_id=source,
                    reset_counter=reset,
                )
                self._candidate = None
            self._identity = identity
            self._sequence, self._stamp = self._watermarks.get(identity, (0, 0))
            source_matches = (
                not self.config.expected_source_id or source == self.config.expected_source_id
            )
            if (
                source_matches
                and identity not in self._watermarks
                and len(self._watermarks) >= STREAM_HISTORY_CAPACITY
            ):
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
                    # 对同一 source_id 遍历所有已确认 reset 周期，求测量时间最高水位。
                    # key[0] 是来源名，watermark[1] 是源纳秒；空历史的 default=0 不代表真实测量时刻。
                    source_stamp = max(
                        (
                            watermark[1]
                            for key, watermark in self._watermarks.items()
                            if key[0] == source
                        ),
                        default=0,
                    )
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
                # 第二项要求错误列表恰好只有 COMMAND_INVALID，不能用“包含该错误”替代。
                # 否则同时含重复序号/坏字段的撤销包也会推进可信历史，破坏后续防重放。
                advance = not errors or (errors == ['COMMAND_INVALID'] and valid is False)
                if advance and source_matches:
                    self._watermarks[identity] = (self._sequence, self._stamp)

    def _receive_freshness(self, age):
        # 只读分类接收年龄 age（秒或 None），返回稳定诊断字符串，不推断源测量年龄。
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
        # 按接收超时→最新错误→是否配置→源年龄的原优先级返回显示字符串。
        #
        # 参数来自当前 snapshot，函数只读，不推进可信水位，不授予缓存或飞行使用权。
        """综合最新消息的错误、接收年龄和源年龄，供诊断窗口展示。"""
        if receive_freshness in ('EXPIRED', 'CLOCK_REGRESSION', 'NO_SAMPLE'):
            return receive_freshness
        if self.latest_errors:
            return 'LATEST_MESSAGE_INVALID'
        time_limits_configured = all(
            (
                self.config.timeout_sec,
                self.config.max_source_age_sec,
                self.config.future_tolerance_sec,
            )
        )
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
        # 以单调秒 now、ROS 纳秒和可选发布者数量重新检查，返回独立诊断快照。
        #
        # 这不是无副作用 getter：会观察时钟、记录过期/数量事件并撤销候选，历史不会续期。
        # 未给 publisher_count 时保持上次观察数量。四阈值和来源绑定齐备等条件全部满足，
        # 才可能 cache_usable=True；即使如此 control_authority 仍严格为 False。
        # 基本时间参数非法抛 ValueError；快照不调用 ROS、不写盘。
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
            last['source_age_sec'] = (
                (ros_now_ns - last['stamp_ns']) / 1e9 if ros_now_ns and last['stamp_ns'] else None
            )
        if self._candidate is None:
            candidate_errors.append('NO_CURRENT_CANDIDATE')
        # all 对四项显式正阈值逐个取与；源绑定与发布者唯一性仍由下面的独立条件检查。
        if not all(getattr(self.config, name) > 0 for name in LIMIT_NAMES):
            candidate_errors.append('LIMITS_UNCONFIGURED')
        if not self.config.expected_source_id:
            candidate_errors.append('SOURCE_BINDING_UNCONFIGURED')
        if self.publisher_count != 1:
            candidate_errors.append('PUBLISHER_COUNT_NOT_ONE')
        freshness = self._message_freshness(receive_freshness, last)
        return _json_safe(
            {
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
            }
        )


class DiagnosticWriter:
    """写入完整快照，并限制日志频率、单文件大小与备份数量。"""

    def __init__(
        self, output_dir='', log_max_bytes=262144, log_backup_count=1, log_min_interval_sec=0.2
    ):
        # 验证日志预算并建立可选输出目录，保存本 writer 独立的节流/待刷状态。
        #
        # output_dir 为空时不创建文件；否则 expanduser 后按需 mkdir，不持有常驻文件描述符。
        # 配置非法抛 ValueError，目录权限/路径等文件系统错误原样传播。
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
        # 记录当前传入的 event，返回该 event 本次是否实际写入；now 为调用者提供的单调秒。
        #
        # 无输出目录返回 False；已有待刷异常先尝试 flush，普通事件在节流期内允许省略。
        # 节流期的异常做深复制，后续异常只合并计数及有限原因，不会被健康尾包冲掉。
        # False 也可能已先 flush 旧待写异常，不能理解为本轮完全没有写盘。
        # 写盘错误原样传播；False 不表示当前 event 已写盘，也不表示没有异常。
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
        # 尝试写出待处理异常，返回是否成功写入；无待写或仍节流时返回 False。
        #
        # force=True 仅跳过节流，不绕过文件系统错误；只有 _write 成功才清待写项。
        # 因此强刷失败后的同一 writer 仍保留异常，可由显式后续调用重试。
        """写出待处理异常；正常退出用 force 跳过节流，写入成功后才清除。"""
        if self._pending_anomaly is None:
            return False
        if (
            not force
            and self.last_logged_at is not None
            and 0 <= now - self.last_logged_at < self.min_interval
        ):
            return False
        written = self._write(self._pending_anomaly, now)
        if written:
            self._pending_anomaly = None
        return written

    def _write(self, event, now):
        # 把一条事件写到有界轮转日志，成功返回 True；文件操作/序列化错误向外传播。
        #
        # 超大记录改为明确的截断事件；从最大备份号向下轮转，防止覆盖尚待搬移的旧备份。
        # 文件写完关闭后才推进节流时刻、清抑制计数；它不承诺 fsync 或突然断电后的持久性。
        value = dict(event, log_events_suppressed=self.suppressed)
        data = (json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False) + '\n').encode()
        if len(data) > self.max_bytes:
            data = (
                json.dumps(
                    dict(
                        kind='event_truncated',
                        event_id=event.get('event_id'),
                        reasons=['LOG_RECORD_TOO_LARGE'],
                        control_authority=False,
                        log_events_suppressed=self.suppressed,
                    )
                )
                + '\n'
            ).encode()
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
        # 将 value 写成完整 JSON 快照；无输出目录直接返回，其余成功也无返回值。
        #
        # 临时文件写完后同目录 os.replace，读者看到旧版或新版完整文件；不承诺断电持久性。
        # finally 尝试删除遗留临时文件，写入/替换/删除错误沿原异常规则传播，不伪称成功。
        if self.directory is None:
            return
        data = json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        descriptor, temporary = tempfile.mkstemp(
            prefix='.velocity_command_diagnostic.', dir=self.directory
        )
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
    value.update(
        topic=topic,
        receiver_running=False,
        stopped_at_monotonic=now,
        stopped_at_ros_ns=ros_now_ns,
        stopped_at_system_ns=system_now_ns,
        shutdown_clock_source='last_observed_ros_clock',
        cache_usable=False,
        current_candidate=None,
    )
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


def _release_diagnostic_resources(actions, primary_error):
    """逐项尝试 main 所有者提供的清理动作，保留主异常并按失败顺序连接清理异常。

    actions 只含无参数清理函数，调用次序由 main 明确列出；本函数不启动 ROS、执行回调
    或改变控制数据。primary_error 是正在向外传播的异常或 None，不能被 destroy/shutdown
    的次生错误覆盖。没有主异常时传播首个清理错误；每项失败也不阻止后续动作。
    """
    errors = []
    for release in actions:
        try:
            release()
        except BaseException as cleanup_error:
            errors.append(cleanup_error)

    failure = primary_error if primary_error is not None else (errors[0] if errors else None)
    if failure is None:
        return
    seen = set()

    def visible_chain_tail(error):
        """沿 traceback 实际显示的 cause/context 找尾部；只剪断指回已见异常的循环边。"""
        while True:
            seen.add(id(error))
            following = error.__cause__
            if following is None and not error.__suppress_context__:
                following = error.__context__
            if following is None:
                return error
            if id(following) in seen:
                if error.__cause__ is following:
                    error.__cause__ = None
                    error.__suppress_context__ = True
                if error.__context__ is following:
                    error.__context__ = None
                return error
            error = following

    tail = visible_chain_tail(failure)
    for cleanup_error in errors:
        if id(cleanup_error) in seen:
            continue
        next_tail = visible_chain_tail(cleanup_error)
        # 不直接覆盖主异常的 cause：原本通过 implicit context 显示的业务首错也要保留。
        # 清理错误由 finally 自动带回主异常的 context 已在上面的遍历中剪掉。
        tail.__cause__ = cleanup_error
        tail.__suppress_context__ = True
        tail = next_tail
    if primary_error is None:
        raise failure


# 只有执行入口才加载 ROS；创建订阅与诊断定时器，不创建飞控 publisher。核心接收器可独立离线测试。
def main(args=None):
    # 以可选 ROS 命令行 args 运行只读诊断；正常停止无返回值，业务/清理失败保持可见。
    #
    # 本入口在主线程持有节点与临时停止处理器，不创建飞控发布器。基类构造成功后立即
    # 保存 node，再配置端点；配置失败也会显式销毁。基类构造未返回时，入口尚未取得
    # 完整 Node，只能依赖底层释放，不保证任意构造中途异常均完整释放。
    # SIGINT/SIGTERM 只请求停止；当前回调先返回，再写终态、移出/销毁节点、关闭自有
    # Context 并恢复原处理器。已有调用者默认 Context 的重复初始化错误不授权本入口关闭它。
    # 100 ms 只限制 executor 空闲等待，不是包含回调/写盘/清理的硬实时退出保证。
    import signal

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor
    from jikl_fc01_system.msg import VelocityCommand

    class DiagnosticNode(Node):
        def __init__(self):
            # 只成功构造 ROS 基类 Node；返回后 main 取得显式销毁责任，暂不创建业务端点。
            super().__init__('fc01_velocity_command_diagnostic')

        def configure_diagnostic(self):
            # 按原参数→接收器→writer→QoS→订阅→定时器→日志→首快照顺序接线。
            #
            # 无参数或返回值；main 必须先持有此完整 Node。后续任一操作失败，main 仍负责销毁。
            # 此时尚未登记 executor，不运行消息/定时器回调；首个 snapshot 是显式同步调用。

            def parameter(name, default):
                # 声明一个启动只读参数并返回原值；读取/声明错误原样传播，不作隐式类型转换。
                return self.declare_parameter(
                    name,
                    default,
                    descriptor=ParameterDescriptor(
                        read_only=True,
                        description='Startup-only diagnostic configuration; restart to change.',
                    ),
                ).value

            self.topic = parameter('topic', DEFAULT_TOPIC)
            depth = parameter('qos_depth', 5)
            reliable = parameter('reliable', False)
            period = parameter('snapshot_period_sec', 1.0)
            if (
                not isinstance(self.topic, str)
                or not self.topic.startswith('/')
                or self.topic.startswith('/fmu/')
                or not _integer(depth, 20)
                or depth < 1
                or not _finite(period)
                or period < 0.01
                or period > 60
            ):
                raise ValueError('invalid diagnostic topic, depth, or snapshot period')
            # vars 读取 DiagnosticConfig 默认字段，推导式按字段顺序声明同名 ROS 参数；
            # ** 把字典展开为具名参数，仍由 DiagnosticConfig 拒绝非法值，不默填生产策略。
            self.receiver = Receiver(
                DiagnosticConfig(
                    **{
                        name: parameter(name, default)
                        for name, default in vars(DiagnosticConfig()).items()
                    }
                )
            )
            self.writer = DiagnosticWriter(
                parameter('output_dir', ''),
                parameter('log_max_bytes', 262144),
                parameter('log_backup_count', 1),
                parameter('log_min_interval_sec', 0.2),
            )
            qos = QoSProfile(
                depth=depth,
                durability=DurabilityPolicy.VOLATILE,
                reliability=(
                    ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
                ),
            )
            self.subscription = self.create_subscription(
                VelocityCommand, self.topic, self.receive, qos
            )
            # ROS 接收与定时快照共用同一 Receiver；没有任何飞控 publisher。
            self.timer = self.create_timer(period, self.snapshot)
            self.get_logger().info(
                'Diagnostic only; control_authority=false type='
                + MESSAGE_TYPE
                + ' topic='
                + self.topic
                + ' zero limits are UNCONFIGURED'
            )
            self.snapshot()

        def receive_context(self):
            # 返回（单调秒、非负 ROS 纳秒、当前发布者数），每次重新查询而不跨时钟比较。
            #
            # 调用 ROS 时钟/图接口可能因外部关闭或其他运行错误失败，交由 main 保留原错并清理。
            # count_publishers 是 ROS 图中的端点数量，不证明 QoS 已匹配或消息来源可信；
            # 仍须由 Receiver 检查每帧来源、顺序和时间，数量恢复也不能让旧候选自动复活。
            return (
                time.monotonic(),
                max(0, self.get_clock().now().nanoseconds),
                self.count_publishers(self.topic),
            )

        def flush_events(self, now):
            # 按队列顺序取走并记录诊断事件；无返回值，writer 错误原样传播。
            #
            # drain_events 已清队列，写入失败不会自动重新入队；main 仍会尝试保存终态和资源释放。
            for event in self.receiver.drain_events():
                self.writer.record(event, now)

        def receive(self, message):
            # 处理一条只读消息回调：先取三项上下文，再接收校验，最后转存事件。
            #
            # 无返回值；异常交 executor/main 处理，不把错误消息替换成飞行指令或控制授权。
            now, ros_now, publishers = self.receive_context()
            self.receiver.receive(message, now, ros_now, publishers)
            self.flush_events(now)

        def snapshot(self):
            # 重新检查过期并写运行快照；无新包也可撤销候选，不能靠显示刷新寿命。
            #
            # 无参数或返回值；先取状态，再依原顺序转存事件/刷待写异常/替换快照。
            # 文件错误不吞掉，也不把这次函数调用解释为飞控执行或导航成功。
            now, ros_now, publishers = self.receive_context()
            value = self.receiver.snapshot(now, ros_now, publishers)
            value['topic'] = self.topic
            value['receiver_running'] = True
            value['log_events_suppressed'] = self.writer.suppressed
            self.flush_events(now)
            self.writer.flush(now)
            self.writer.snapshot(value)

    stop_requested = False

    def request_stop(_signum, _frame):
        # 仅将 main 局部停止标志置真；重复信号幂等，不执行 ROS、写盘或资源释放。
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    executor = None
    configured = False
    context_owned = False
    primary_error = None
    try:
        for signum in previous_handlers:
            signal.signal(signum, request_stop)
        # 只关闭本入口从未初始化状态取得的默认 context；已有调用者 context 的重复 init 原样报错。
        context_owned = not rclpy.ok()
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = DiagnosticNode()
        node.configure_diagnostic()
        # 此位只表示参数、端点和首个同步快照均完成，可在退出时保存完整终态；不授予控制权。
        configured = True
        executor = rclpy.get_global_executor()
        executor.add_node(node)
        # 与原 rclpy.spin 一样串行执行一项 executor 工作；停止信号不在回调中途关闭 context。
        while executor.context.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as error:
        primary_error = error
        raise
    finally:
        stop_requested = True
        # 顺序就是资源合同：终态文件不查询 ROS -> 移出 executor -> 销毁 Node -> 关自有 Context -> 恢复信号。
        # 列表里只保存本 finally 内同步执行的无参函数，不把 node/executor 引用留到下一次 main。
        actions = []
        if configured:
            actions.append(
                lambda: finalize_diagnostic(
                    node.receiver, node.writer, node.topic, time.monotonic(), time.time_ns()
                )
            )
        if executor is not None and node is not None:
            actions.append(lambda: executor.remove_node(node))
        if node is not None:
            actions.append(node.destroy_node)
        if context_owned:
            actions.append(lambda: rclpy.shutdown() if rclpy.ok() else None)
        # 默认参数立即绑定本轮 signum/handler；若只写无参 lambda，会在稍后调用时都读到循环末项。
        for signum, handler in previous_handlers.items():
            actions.append(lambda signum=signum, handler=handler: signal.signal(signum, handler))
        _release_diagnostic_resources(actions, primary_error)


if __name__ == '__main__':
    main()
