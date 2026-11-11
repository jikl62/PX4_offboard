#!/usr/bin/env python3
# 视觉实际状态的独立离线验收层：在适配转换外检查时间、可信周期、质量、协方差与连续性。
# 本文件不生成目标指令，也不把诊断 accepted 当飞行授权；拒绝撤销当前缓存但保留可信历史。
"""视觉实际状态的离线校验：先检查来源和时间，再检查测量，最后提交可信缓存。

输入是严格配对的 Odometry 和 VisionHealth，表示飞机“实际在哪里、如何运动”。
目标速度不经过这里。本模块复用原坐标转换，不创建 ROS 节点或飞控发布者，
也不估算时钟偏移、变换协方差。accepted 只表示通过本次离线检查，不授予飞行权限。
"""
from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass, field, replace
import importlib.util
import math
from pathlib import Path
import sys
from typing import Optional


def _load_adapter():
    # 源码目录和安装目录都将转换器放在旁边，始终复用同一份转换实现。
    name = '_fc01_state_validation_odometry_adapter'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            name, Path(__file__).with_name('vision_odometry_adapter.py')
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


adapter = _load_adapter()
AdapterConfig = adapter.AdapterConfig
ConvertedState = adapter.ConvertedState


def _load_resource_budget():
    """加载与相邻接收工具共享的 UTF-8 字节预算，不依赖启动工作目录。"""
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


# 只接受有限数且排除 bool；超大整数转换溢出同样按无效数处理。
def _number(value):
    try:
        return (
            isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        )
    except OverflowError:
        return False


# 协议整数限制范围，排除 bool 冒充计数或时间字段。
def _integer(value, maximum):
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= maximum


# ROS 时间须在 sec/nanosec 约定范围，零时刻表示未提供；精确整数纳秒用于测量配对。
def _stamp(value):
    if value is None:
        return None
    sec, ns = getattr(value, 'sec', None), getattr(value, 'nanosec', None)
    if not _integer(sec, 2**31 - 1) or not _integer(ns, 999999999):
        return None
    return sec * 1000000000 + ns or None


def _attitude_step_rad(current, previous):
    """计算两个四元数之间的转角，保留接近零的真实变化。"""
    # q 和 -q 表示同一姿态，先选择与基线同号的表示。
    dot = sum(a * b for a, b in zip(current, previous))
    aligned = tuple(-value for value in current) if dot < 0 else current
    # acos(dot) 在 dot 接近 1 时会放大舍入误差，甚至将相同姿态判成跳变。
    # 用差、和向量的长度求半角关系，相同输入的差严格为零。
    difference = math.dist(aligned, previous)
    total = math.hypot(*(a + b for a, b in zip(aligned, previous)))
    return 4.0 * math.atan2(difference, total)


@dataclass(frozen=True)
class ClockPolicy:
    """显式时间对应：本机 ROS 时间 = 视觉测量时间 + offset_ns。"""

    # confirmed=True 之前不应用偏移；已确认同一时钟时也要显式填写 0。
    offset_ns: Optional[int] = None
    confirmed: bool = False

    def __post_init__(self):
        if type(self.confirmed) is not bool:
            raise ValueError('clock confirmed must be bool')
        if self.offset_ns is not None and (
            not isinstance(self.offset_ns, int) or isinstance(self.offset_ns, bool)
        ):
            raise ValueError('clock offset_ns must be an integer or None')
        if self.confirmed and self.offset_ns is None:
            raise ValueError('confirmed clock requires an explicit offset, including zero')


@dataclass(frozen=True)
class ValidationConfig:
    """校验策略。None 表示该条件尚未确认，不能用猜测值填补。"""

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
    # 原始 Odometry 坐标系中六个方差的上限，不是转换后坐标系的协方差。
    max_pose_variance: Optional[tuple] = None
    max_twist_variance: Optional[tuple] = None
    # 历史容量只限制资源使用；满后拒绝新周期，不淘汰旧周期的防回放证据。
    epoch_history_capacity: int = 128

    def __post_init__(self):
        self.adapter.validate()
        # frozen 只能阻止字段重新赋值，不能阻止调用方原地修改传入的列表。
        # 将四组外参和对齐数据保存为独立数值元组，使可信周期的转换保持不变。
        transform_fields = (
            'body_from_sensor_wxyz',
            'sensor_position_body_frd',
            'world_alignment_wxyz',
            'world_translation_ned',
        )
        transforms = {
            name: tuple(float(value) for value in getattr(self.adapter, name))
            for name in transform_fields
        }
        object.__setattr__(self, 'adapter', replace(self.adapter, **transforms))
        identity_fields = (
            'source_id',
            'input_frame_id',
            'input_body_frame_id',
            'extrinsics_id',
            'alignment_id',
        )
        for name in identity_fields:
            if not resource_budget.identity_within_budget(getattr(self.adapter, name)):
                raise ValueError('adapter identities must be valid UTF-8 within 256 bytes')
        if not isinstance(self.clock, ClockPolicy):
            raise ValueError('explicit ClockPolicy required')
        if not _integer(self.epoch_history_capacity, 4096) or self.epoch_history_capacity < 1:
            raise ValueError('epoch_history_capacity must be an integer in [1,4096]')

        positive_limits = (
            'receive_timeout_sec',
            'max_source_age_sec',
            'future_tolerance_sec',
            'max_position_step_m',
            'max_velocity_step_mps',
            'max_attitude_step_rad',
        )
        for name in positive_limits:
            value = getattr(self, name)
            if value is None:
                continue
            if not _number(value) or value < 0 or (value == 0 and name != 'future_tolerance_sec'):
                raise ValueError(
                    name + ' must be finite and positive ' '(future tolerance may be zero), or None'
                )
        for name in ('min_quality', 'nominal_quality_min'):
            value = getattr(self, name)
            if value is not None and (not _number(value) or not 0 <= value <= 1):
                raise ValueError(name + ' must be in [0,1] or None')
        if (
            self.min_quality is not None
            and self.nominal_quality_min is not None
            and self.nominal_quality_min < self.min_quality
        ):
            raise ValueError('nominal quality must not be below minimum quality')

        for name in ('max_pose_variance', 'max_twist_variance'):
            value = getattr(self, name)
            if value is None:
                continue
            if (
                not isinstance(value, (tuple, list))
                or len(value) != 6
                or not all(_number(x) and x >= 0 for x in value)
            ):
                raise ValueError(name + ' must contain six finite nonnegative raw variances')
            object.__setattr__(self, name, tuple(value))


@dataclass(frozen=True)
class SampleMetadata:
    """本地明确提供的声明，不增加 ROS 消息字段。"""

    pose_covariance_valid: bool = False
    twist_covariance_valid: bool = False
    relocalization_id: str = ''

    def __post_init__(self):
        if (
            type(self.pose_covariance_valid) is not bool
            or type(self.twist_covariance_valid) is not bool
        ):
            raise ValueError('covariance declarations must be bool')
        if not resource_budget.identity_within_budget(self.relocalization_id):
            raise ValueError('relocalization_id must be valid UTF-8 within 256 bytes')


def covariance_diagnostic(values, declared, upper=None):
    """检查原始 6×6 协方差：有限值、方差、对称性、半正定性和显式上限。

    半正定表示任意方向的方差都不为负；只检查六个对角线还不够。
    下面的 tolerance 只容许矩阵计算舍入误差，不是飞机可接受的不确定性。
    检查通过也不会把输出标记为“已完成协方差坐标变换”。
    """
    if not declared:
        return dict(status='UNKNOWN', reasons=[], variances=None, transformed_valid=False)
    try:
        entries = tuple(values)
    except TypeError:
        entries = ()
    if len(entries) != 36 or not all(_number(x) for x in entries):
        return dict(
            status='INVALID', reasons=['COVARIANCE_NONFINITE_OR_WRONG_SIZE'], variances=None
        )
    diagonal = tuple(entries[i * 6 + i] for i in range(6))
    reasons = []
    if any(x < 0 for x in diagonal):
        reasons.append('COVARIANCE_NEGATIVE_VARIANCE')
    scale = max((abs(x) for x in entries), default=0.0)
    # 每个元素分别除以两个轴的标准差，得到无量纲相关矩阵。
    # 这样某一轴的极大方差不能掩盖其他轴中的错误；分两次除法避免乘积溢出。
    residual = [[0.0] * 6 for _ in range(6)]
    tolerance = 1.0e-10
    if not reasons:
        standard_deviations = tuple(math.sqrt(x) for x in diagonal)
        for i in range(6):
            for j in range(6):
                value = entries[i * 6 + j]
                if standard_deviations[i] == 0 or standard_deviations[j] == 0:
                    if value != 0:
                        reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                    continue
                larger_deviation = max(standard_deviations[i], standard_deviations[j])
                smaller_deviation = min(standard_deviations[i], standard_deviations[j])
                normalized = (value / larger_deviation) / smaller_deviation
                if not math.isfinite(normalized) or abs(normalized) > 1.0 + tolerance:
                    reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                residual[i][j] = normalized
        if any(
            abs(residual[i][j] - residual[j][i]) > tolerance for i in range(6) for j in range(i)
        ):
            reasons.append('COVARIANCE_NOT_SYMMETRIC')
    if not reasons:
        # 在副本上做带主元的半正定 Cholesky 检查：逐列消去已解释的相关项。
        # 剩余方差不能为负；若主元为零，剩余整块也必须接近零。
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
                if any(
                    abs(residual[i][j]) > tolerance
                    for i in range(column, 6)
                    for j in range(column, 6)
                ):
                    reasons.append('COVARIANCE_NOT_POSITIVE_SEMIDEFINITE')
                break
            for i in range(column + 1, 6):
                for j in range(i, 6):
                    updated = residual[i][j] - (residual[i][column] / value) * residual[j][column]
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
    return dict(
        status=status,
        reasons=list(dict.fromkeys(reasons)),
        variances=diagonal,
        transformed_valid=False,
    )


@dataclass(frozen=True)
class ValidationResult:
    """对外返回副本，调用方修改结果不能污染内部可信基线或缓存。"""

    state: ConvertedState
    accepted: bool
    diagnostics: dict
    control_authority: bool = field(default=False, init=False)


class Validator:
    """校验实际状态并维护可恢复的当前缓存，以及独立的可信历史。

    epoch 是来源、坐标、reset 和重定位身份组成的测量周期。新周期允许重建
    连续性基线，但不能绕过来源/坐标配置或使已确认的测量时钟倒退。
    拒绝、过期只撤销当前缓存，保留可信基线和历史，避免异常样本洗成正常数据。
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
        self._epoch = None  # 最近观察的周期，可能来自被拒绝的消息。
        self._accepted_epoch = None  # 最近真正接受的周期，用于序号和连续性检查。
        self._trusted_epochs = set()  # 已接受过的所有周期，防止回放退役周期。
        self._sequence = 0
        # 周期切换可以重启序号；测量时间只能由真正接受的样本推进。
        self._source_stamp = 0
        self._last_receive = self._last_ros = None
        self._baseline = self._cache = None  # 可信连续性基线 / 此刻可用的样本。
        self._last_reasons = ('NO_SAMPLE',)
        self._previously_rejected = False

    # 事件与原因计数独立累计，队列有界；last_anomaly 保存副本，后续正常样本和调用方改动都不覆盖它。
    def _event(self, kind, reasons, **fields):
        self._event_id += 1
        value = dict(
            event_id=self._event_id,
            kind=kind,
            reasons=list(reasons),
            control_authority=False,
            **fields,
        )
        if len(self._events) == self._events.maxlen:
            self.counters['events_dropped'] += 1
        self._events.append(value)
        self.counters[kind] += 1
        self.reason_counts.update(reasons)
        if kind in ('rejected', 'expired', 'clock_regression'):
            self.last_anomaly = deepcopy(value)
        return value

    # 向调用者交付深拷贝再清事件队列；可信缓存、基线和最后异常继续保留。
    def drain_events(self):
        values = deepcopy(list(self._events))
        self._events.clear()
        return values

    def _observe_clocks(self, now, ros_now_ns):
        # 接收和查询共享时钟水位；一次倒退不能把后续比较基准一起拉回去。
        reasons = []
        if self._last_receive is not None and now < self._last_receive:
            reasons.append('RECEIVE_CLOCK_REGRESSION')
        if self._last_ros is not None and ros_now_ns < self._last_ros:
            reasons.append('ROS_CLOCK_REGRESSION')
        self._last_receive = now if self._last_receive is None else max(now, self._last_receive)
        self._last_ros = ros_now_ns if self._last_ros is None else max(ros_now_ns, self._last_ros)
        return reasons

    # 缓存寿命同时受单调接收年龄和映射后源测量年龄约束；刚接收不代表源数据仍然新鲜。
    def _cache_age_errors(self, now, ros_now_ns):
        if self._cache is None:
            return [], None
        state, received, _, _ = self._cache
        source_age = (ros_now_ns - state.stamp_ns) / 1e9 if ros_now_ns else None
        reasons = []
        if now - received >= self.config.receive_timeout_sec:
            reasons.append('RECEIVE_SAMPLE_EXPIRED')
        if source_age is None:
            reasons.append('SOURCE_CLOCK_UNAVAILABLE')
        elif (
            source_age >= self.config.max_source_age_sec
            or source_age < -self.config.future_tolerance_sec
        ):
            reasons.append('SOURCE_SAMPLE_OUTSIDE_TIME_WINDOW')
        return reasons, source_age

    # 将未明确配置的验收条件列为 unknown；不会用经验值补齐或把未检查内容写成已确认。
    def _unknowns(self):
        names = (
            'receive_timeout_sec',
            'max_source_age_sec',
            'future_tolerance_sec',
            'min_quality',
            'nominal_quality_min',
            'max_position_step_m',
            'max_velocity_step_mps',
            'max_attitude_step_rad',
            'max_pose_variance',
            'max_twist_variance',
        )
        unknown = [name for name in names if getattr(self.config, name) is None]
        if not self.config.clock.confirmed:
            unknown.append('clock_offset')
        return unknown

    # 结果深拷贝状态、协方差及异常证据；诊断包含未知项与可信周期容量，control_authority 始终 false。
    def _result(self, state, accepted, reasons, tracking='UNKNOWN', covariance=None, **fields):
        diagnostics = dict(
            control_authority=False,
            qualification='DIAGNOSTIC_ONLY',
            reasons=list(reasons),
            unknown=self._unknowns(),
            tracking_status=tracking,
            covariance=deepcopy(covariance) if covariance is not None else {},
            counters=dict(self.counters),
            reason_counts=dict(self.reason_counts),
            trusted_epoch_count=len(self._trusted_epochs),
            epoch_history_capacity=self.config.epoch_history_capacity,
            last_anomaly=deepcopy(self.last_anomaly),
            event_id=self._event_id,
            **fields,
        )
        return ValidationResult(deepcopy(state), accepted, diagnostics)

    # 拒绝结果使用无有效位的新状态与首个原因；不把上一次可信测量伪装成本轮测量。
    def _invalid(self, reasons, **fields):
        return self._result(
            ConvertedState(
                source_id=self.config.adapter.source_id, reason=reasons[0] if reasons else 'INVALID'
            ),
            False,
            reasons,
            **fields,
        )

    # 最近观测周期与最近接受周期分别保存；身份变化先记录，只有 receive 最终通过才能推进可信历史。
    def _observe_epoch(self, raw, health, metadata):
        if health is None:
            return None
        source, reset = getattr(health, 'source_id', None), getattr(health, 'reset_counter', None)
        frame = getattr(getattr(raw, 'header', None), 'frame_id', None)
        body = getattr(raw, 'child_frame_id', None)
        if not all(
            resource_budget.identity_within_budget(x) and x for x in (source, frame, body)
        ) or not _integer(reset, 2**32 - 1):
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

    def _epoch_errors(self, epoch, same_epoch):
        """只检查周期是否允许；真正接受样本后才将其加入可信历史。"""
        reasons = []
        if epoch is None:
            reasons.append('EPOCH_IDENTITY_INVALID')
        elif not same_epoch:
            if epoch in self._trusted_epochs:
                reasons.append('RETIRED_EPOCH_REPLAY')
            elif len(self._trusted_epochs) >= self.config.epoch_history_capacity:
                reasons.append('EPOCH_HISTORY_CAPACITY_REACHED')
        return reasons

    def _health_and_sequence_errors(self, health, sequence, same_epoch):
        """检查健康消息字段和本周期序号，不推进任何可信水位。"""
        reasons = []
        if type(getattr(health, 'schema_version', None)) is not int or health.schema_version != 1:
            reasons.append('HEALTH_VERSION_INVALID')
        if not _integer(getattr(health, 'reset_counter', None), 2**32 - 1):
            reasons.append('RESET_COUNTER_INVALID')
        if not _number(getattr(health, 'quality', None)):
            reasons.append('QUALITY_NOT_FINITE_NUMBER')
        if health is not None:
            flags = (
                'reset_valid',
                'initialized',
                'tracking',
                'calibration_valid',
                'frame_alignment_valid',
                'time_sync_valid',
                'extrinsics_valid',
                'metric_scale_valid',
                'quality_valid',
                'position_valid',
                'velocity_valid',
                'attitude_valid',
                'angular_velocity_valid',
            )
            if any(
                hasattr(health, name) and type(getattr(health, name)) is not bool for name in flags
            ):
                reasons.append('HEALTH_CONFIRMATION_NOT_BOOL')
        if not _integer(sequence, 2**64 - 1) or not sequence:
            reasons.append('SEQUENCE_INVALID')
        elif same_epoch:
            if sequence == self._sequence:
                reasons.append('DUPLICATE_SEQUENCE')
            elif sequence < self._sequence:
                reasons.append('SEQUENCE_REGRESSION')
        return reasons

    def _measurement_time(self, raw_stamp, health_stamp, ros_now_ns):
        """将测量时间映射到本机 ROS 时域，保留配对和年龄检查的原因。"""
        reasons = []
        if raw_stamp is None or health_stamp is None or raw_stamp != health_stamp:
            reasons.append('INVALID_OR_UNPAIRED_STAMP')
        elif raw_stamp <= self._source_stamp:
            reasons.append('MEASUREMENT_CLOCK_NOT_ADVANCING')
        clock = self.config.clock
        if not clock.confirmed:
            reasons.append('CLOCK_OFFSET_UNCONFIRMED')
        if any(
            getattr(self.config, key) is None
            for key in ('receive_timeout_sec', 'max_source_age_sec', 'future_tolerance_sec')
        ):
            reasons.append('TIMING_POLICY_UNCONFIGURED')
        mapped_stamp = (
            raw_stamp + clock.offset_ns if raw_stamp is not None and clock.confirmed else None
        )
        if mapped_stamp is not None and not 0 < mapped_stamp <= (2**31) * 1000000000 - 1:
            reasons.append('MAPPED_STAMP_OUT_OF_RANGE')
            mapped_stamp = None
        source_age = (ros_now_ns - mapped_stamp) / 1e9 if ros_now_ns and mapped_stamp else None
        if source_age is not None:
            if (
                self.config.max_source_age_sec is not None
                and source_age >= self.config.max_source_age_sec
            ):
                reasons.append('SOURCE_SAMPLE_EXPIRED')
            if (
                self.config.future_tolerance_sec is not None
                and source_age < -self.config.future_tolerance_sec
            ):
                reasons.append('SOURCE_SAMPLE_IN_FUTURE')
        elif clock.confirmed:
            reasons.append('SOURCE_CLOCK_UNAVAILABLE')
        return mapped_stamp, source_age, reasons

    def _check_covariances(self, raw, metadata):
        """只有调用方显式声明有效时，才检查原始位姿/速度协方差。"""
        covariance = {}
        reasons = []
        for name, declared, upper in (
            ('pose', metadata.pose_covariance_valid, self.config.max_pose_variance),
            ('twist', metadata.twist_covariance_valid, self.config.max_twist_variance),
        ):
            values = getattr(getattr(raw, name, None), 'covariance', None)
            value = covariance_diagnostic(values, declared, upper)
            covariance[name] = value
            reasons.extend(name.upper() + '_' + x for x in value['reasons'])
            if not declared and upper is not None:
                reasons.append(name.upper() + '_COVARIANCE_UNKNOWN_WITH_LIMIT')
        return covariance, reasons

    def _convert_measurement(self, raw, health, raw_stamp, health_stamp, mapped_stamp):
        """复用既有转换器；时间偏移只写入副本，不修改输入消息。"""
        reasons = []
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
        return converted, reasons

    def _tracking_status(self, health, converted):
        """区分丢失、未分类、正常和降级；质量最低要求由显式策略决定。"""
        reasons = []
        tracking = 'UNKNOWN'
        if getattr(health, 'tracking', None) is False:
            tracking = 'LOST'
        elif converted.valid_fields & adapter.QUALITY:
            threshold = self.config.nominal_quality_min
            if threshold is None:
                tracking = 'TRACKING_UNCLASSIFIED'
            elif converted.quality >= threshold:
                tracking = 'NORMAL'
            else:
                tracking = 'DEGRADED'
            if self.config.min_quality is not None and converted.quality < self.config.min_quality:
                reasons.append('QUALITY_BELOW_EXPLICIT_MINIMUM')
        return tracking, reasons

    def _continuity_errors(self, converted, same_epoch):
        """同一可信周期内比较位置、速度和姿态；新周期重新建立基线。"""
        if not same_epoch or self._baseline is None:
            return []
        if converted.valid_fields & adapter.REQUIRED != adapter.REQUIRED:
            return []
        changes = (
            (
                'POSITION',
                math.dist(converted.position, self._baseline.position),
                self.config.max_position_step_m,
            ),
            (
                'VELOCITY',
                math.dist(converted.velocity, self._baseline.velocity),
                self.config.max_velocity_step_mps,
            ),
            (
                'ATTITUDE',
                _attitude_step_rad(converted.attitude, self._baseline.attitude),
                self.config.max_attitude_step_rad,
            ),
        )
        return [
            name + '_CONTINUITY_LIMIT_EXCEEDED'
            for name, distance, limit in changes
            if limit is not None and distance > limit
        ]

    def receive(self, raw, health, received_at, ros_now_ns, metadata=None):
        """按顺序校验一对实际状态消息；所有检查结束后才更新可信数据。"""
        metadata = metadata or SampleMetadata()
        if not isinstance(metadata, SampleMetadata):
            raise ValueError('SampleMetadata required')
        if not _number(received_at) or received_at < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('independent receive clocks must be finite and nonnegative')
        self.counters['received'] += 1

        # 1. 先观察旧缓存的寿命。即使新包身份非法或随后恢复正常，旧超时也不能漏记。
        reasons = self._observe_clocks(received_at, ros_now_ns)
        if not reasons:
            expired, _ = self._cache_age_errors(received_at, ros_now_ns)
            if expired:
                self._cache = None
                self._previously_rejected = True
                self._event('expired', expired, observed_at=received_at, observed_on_receive=True)

        # 身份资源检查必须早于周期事件、转换和深拷贝。异常包包括迟到包均撤销旧缓存，
        # 但不占用可信历史，也不把原始超长字段放进原因或日志中。
        identity_error = resource_budget.odometry_identity_error(
            raw
        ) or resource_budget.health_identity_error(health)
        if identity_error:
            reasons.append(identity_error)
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(reasons)
            self._event('rejected', reasons, received_at=received_at)
            return self._invalid(reasons, freshness='INVALID')

        # 2. 校验来源周期、健康字段、序号及测量时间。这里只观察，不提交可信历史。
        epoch = self._observe_epoch(raw, health, metadata)
        same_epoch = epoch is not None and epoch == self._accepted_epoch
        reasons.extend(self._epoch_errors(epoch, same_epoch))
        raw_stamp = _stamp(getattr(getattr(raw, 'header', None), 'stamp', None))
        health_stamp = _stamp(getattr(health, 'stamp', None))
        sequence = getattr(health, 'sequence', None)
        reasons.extend(self._health_and_sequence_errors(health, sequence, same_epoch))
        mapped_stamp, source_age, time_errors = self._measurement_time(
            raw_stamp, health_stamp, ros_now_ns
        )
        reasons.extend(time_errors)

        # 3. 检查测量内容。协方差只检查原始坐标，实际状态通过既有转换器对齐。
        covariance, covariance_errors = self._check_covariances(raw, metadata)
        reasons.extend(covariance_errors)
        converted, conversion_errors = self._convert_measurement(
            raw, health, raw_stamp, health_stamp, mapped_stamp
        )
        reasons.extend(conversion_errors)
        tracking, tracking_errors = self._tracking_status(health, converted)
        reasons.extend(tracking_errors)
        reasons.extend(self._continuity_errors(converted, same_epoch))
        reasons = list(dict.fromkeys(reasons))

        # 4. 拒绝只撤销当前缓存，保留最后可信基线。通过后一次性提交新可信数据。
        if reasons:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(reasons)
            self._event('rejected', reasons, received_at=received_at, raw_stamp_ns=raw_stamp)
            return self._invalid(
                reasons,
                tracking=tracking,
                covariance=covariance,
                source_age_sec=source_age,
                raw_stamp_ns=raw_stamp,
                converted_valid_fields=converted.valid_fields,
                freshness='INVALID',
            )
        if self._previously_rejected:
            self._event('recovered', [], received_at=received_at)
        self._previously_rejected = False
        self._accepted_epoch = epoch
        self._trusted_epochs.add(epoch)
        self._sequence, self._source_stamp = sequence, raw_stamp
        self._baseline = deepcopy(converted)
        self._cache = (deepcopy(converted), received_at, tracking, deepcopy(covariance))
        self._last_reasons = ()
        self._event('accepted', [], received_at=received_at, raw_stamp_ns=raw_stamp)
        return self._result(
            converted,
            True,
            [],
            tracking,
            covariance,
            source_age_sec=source_age,
            raw_stamp_ns=raw_stamp,
            freshness='CURRENT',
        )

    def current(self, now, ros_now_ns):
        """重新检查当前缓存；历史接受过不等于此刻仍然有效。"""
        if not _number(now) or now < 0 or not _integer(ros_now_ns, 2**64 - 1):
            raise ValueError('independent current clocks must be finite and nonnegative')
        clock_errors = self._observe_clocks(now, ros_now_ns)
        if clock_errors:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(clock_errors)
            self._event('clock_regression', clock_errors, observed_at=now)
            return self._invalid(clock_errors, freshness='INVALID')
        if self._cache is None:
            return self._invalid(
                self._last_reasons or ('NO_CURRENT_SAMPLE',), freshness='NO_CURRENT_SAMPLE'
            )
        state, received, tracking, covariance = self._cache
        reasons, source_age = self._cache_age_errors(now, ros_now_ns)
        if reasons:
            self._cache = None
            self._previously_rejected = True
            self._last_reasons = tuple(reasons)
            self._event('expired', reasons, observed_at=now)
            return self._invalid(reasons, freshness='INVALID', source_age_sec=source_age)
        return self._result(
            state,
            True,
            [],
            tracking,
            covariance,
            receive_age_sec=now - received,
            source_age_sec=source_age,
            freshness='CURRENT',
        )
