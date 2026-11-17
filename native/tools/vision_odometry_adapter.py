#!/usr/bin/env python3
# 实际状态适配层：输入 Odometry+VisionHealth，输出 VisionState；从不将目标速度解释成自身测量。
# 坐标转换与配对保持独立；ROS 端按启动合同、端点接线、状态消息与退出生命周期分层。
"""将严格配对的 Odometry / VisionHealth 转换成 FC01 的实际状态。

阅读顺序：convert_odometry() 做坐标转换；StampPairBuffer 按测量时间配对；
GuardedOdometryAdapter 检查两个原始话题的来源数量；main() 才连接 ROS。
导入本文件不会启动节点、访问硬件或发布。输出是实际测量状态，不是目标指令；
协方差完成坐标与原点变换前保持不可用，不能把未实现的结果写成零方差。
"""
# 实际 ROS 接线只运行本文件的 GuardedOdometryAdapter；不会自动调用相邻文件中的
# 离线 Validator。离线的时钟偏移、连续性和协方差诊断不能算作 ROS 路径已执行的检查。
from collections import OrderedDict
from dataclasses import dataclass, field
import importlib.util
import math
from pathlib import Path
import sys
import time


def _load_resource_budget():
    """加载相邻工具共用的 UTF-8 字节预算；导入不启动 ROS，不依赖当前工作目录。"""
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

# 有效位沿用 VisionState 协议；数值为零与“这个物理量有效”是两件不同的事。
POSITION, VELOCITY, ATTITUDE, ANGULAR_VELOCITY = 1, 2, 4, 8
POSE_COVARIANCE, TWIST_COVARIANCE, QUALITY = 16, 32, 64
REQUIRED = POSITION | VELOCITY | ATTITUDE | QUALITY
NAN = math.nan
# 世界系 NED 为北/东/下；机体系 FRD 为前/右/下。以下矩阵只改变轴约定。
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ENU_TO_NED = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
FLU_TO_FRD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def stamp_ns(stamp):
    # 统一成整数纳秒供精确配对；非法时间返回 0，后续不能将其作为有效测量。
    sec, ns = getattr(stamp, 'sec', -1), getattr(stamp, 'nanosec', -1)
    if (
        not isinstance(sec, int)
        or not isinstance(ns, int)
        or sec < 0
        or not 0 <= ns < 1_000_000_000
    ):
        return 0
    return sec * 1_000_000_000 + ns


def _finite(values, count):
    # 原始向量和计算结果都经过同一有限数检查，避免溢出后仍设置有效位。
    try:
        result = tuple(float(x) for x in values)
        return result if len(result) == count and all(math.isfinite(x) for x in result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _quaternion(values):
    # 四元数顺序为 [w,x,y,z]；只对接近单位长度的输入归一化，不修补零姿态。
    q = _finite(values, 4)
    if q is None:
        return None
    norm = math.sqrt(sum(x * x for x in q))
    # 5% 是既有输入模长容差；本轮只说明它，不放宽或修改姿态验收范围。
    return tuple(x / norm for x in q) if norm and abs(norm - 1.0) <= 0.05 else None


def _matrix(q):
    # 单位四元数转换成 3×3 旋转矩阵；保留原有乘加顺序，供后续链式旋转使用。
    w, x, y, z = q
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _transpose(m):
    # 对旋转矩阵，转置表示反向旋转，用来反向应用机体/传感器外参。
    return tuple(zip(*m))


def _mul(a, b):
    # 按 a(b(v)) 的顺序复合旋转；不能交换两矩阵来简化坐标转换。
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _rotate(m, v):
    # 将三维向量 v 旋转到矩阵 m 的目标坐标系，不改变其物理单位。
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _cross(a, b):
    # 叉乘用于计算角速度×杆臂带来的传感器线速度，结果单位为 m/s。
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _matrix_quaternion(m):
    # 选择平方分量最大的分支，使接近 180° 的姿态也不靠接近零的分量作除数。
    candidates = (
        1.0 + m[0][0] + m[1][1] + m[2][2],
        1.0 + m[0][0] - m[1][1] - m[2][2],
        1.0 - m[0][0] + m[1][1] - m[2][2],
        1.0 - m[0][0] - m[1][1] + m[2][2],
    )
    branch = max(range(4), key=lambda i: candidates[i])
    s = 2.0 * math.sqrt(max(0.0, candidates[branch]))
    if branch == 0:
        q = (
            s / 4.0,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        )
    elif branch == 1:
        q = (
            (m[2][1] - m[1][2]) / s,
            s / 4.0,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        )
    elif branch == 2:
        q = (
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            s / 4.0,
            (m[1][2] + m[2][1]) / s,
        )
    else:
        q = (
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            s / 4.0,
        )
    # 输出继续使用单位四元数，并选择 w 非负的等价表示，保持原有协议结果。
    norm = math.sqrt(sum(x * x for x in q))
    return tuple(x / norm * (-1.0 if q[0] < 0.0 else 1.0) for x in q)


def _vector(message):
    # 字段缺失时保留未知值，不能把缺少的轴伪造成静止的零值。
    return _finite(
        (
            getattr(message, 'x', NAN),
            getattr(message, 'y', NAN),
            getattr(message, 'z', NAN),
        ),
        3,
    )


@dataclass(frozen=True)
# 显式接口合同：传感器外参将传感器 FRD 映射到机体 FRD，世界对齐将约定 NED 映射到 FC01 NED。
class AdapterConfig:
    # 来源和原始坐标名称由接口双方显式约定，用来拒绝接错话题或旧标定。
    source_id: str
    input_frame_id: str
    input_body_frame_id: str
    world_convention: str
    body_convention: str
    # R_B_S：传感器 FRD → 机体 FRD 的旋转，[w,x,y,z]；平移为机体系杆臂，m。
    body_from_sensor_wxyz: tuple
    sensor_position_body_frd: tuple
    # 先统一 ENU/NED 约定，再应用世界对齐旋转及 NED 平移（m），不自动估算对齐。
    world_alignment_wxyz: tuple
    world_translation_ned: tuple
    extrinsics_id: str
    alignment_id: str

    def validate(self):
        # 每一份状态都必须能匹配具体来源、坐标、安装外参和世界对齐身份。
        for name in (
            'source_id',
            'input_frame_id',
            'input_body_frame_id',
            'extrinsics_id',
            'alignment_id',
        ):
            if not resource_budget.identity_within_budget(getattr(self, name)):
                raise ValueError(resource_budget.RESOURCE_ERROR + ': ' + name)
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError('explicit ' + name + ' is required')
        if self.world_convention not in ('NED', 'ENU') or self.body_convention not in (
            'FRD',
            'FLU',
        ):
            # 不从 frame 名称猜坐标约定；轴方向错误会直接改变速度和姿态方向。
            raise ValueError('explicit world ENU/NED and body FLU/FRD conventions are required')
        if (
            _quaternion(self.body_from_sensor_wxyz) is None
            or _quaternion(self.world_alignment_wxyz) is None
        ):
            # 外参和对齐旋转必须本身有效，不能用默认零值代替未完成标定。
            raise ValueError(
                'explicit finite unit extrinsic/alignment wxyz quaternions are required'
            )
        if (
            _finite(self.sensor_position_body_frd, 3) is None
            or _finite(self.world_translation_ned, 3) is None
        ):
            raise ValueError('explicit finite extrinsic/alignment translations are required')


@dataclass
# 转换结果同时携带数据与有效位：NaN/零有效位明确表示未知，失败也保留原因供下游撤销旧缓存。
class ConvertedState:
    schema_version: int = 1
    sequence: int = 0
    # 对外序号属于适配器输出流；此字段另外保留上游健康序号，供诊断追踪。
    raw_health_sequence: int = 0
    stamp_ns: int = 0
    reset_counter: int = 0
    frame_id: str = 'fc01_vision_ned'
    body_frame_id: str = 'base_link_frd'
    source_id: str = ''
    angular_velocity_source: str = ''
    initialized: bool = False
    tracking: bool = False
    calibration_valid: bool = False
    valid_fields: int = 0  # 只有完成对应转换后才能置位；默认所有测量均未知。
    # 位置为世界 NED 的 m；线速度为世界 NED 的 m/s；姿态为机体 FRD → 世界 NED。
    position: tuple = field(default_factory=lambda: (NAN,) * 3)
    velocity: tuple = field(default_factory=lambda: (NAN,) * 3)
    attitude: tuple = field(default_factory=lambda: (NAN,) * 4)
    angular_velocity: tuple = field(default_factory=lambda: (NAN,) * 3)  # 机体 FRD，rad/s。
    pose_covariance: tuple = field(default_factory=lambda: (NAN,) * 36)
    twist_covariance: tuple = field(default_factory=lambda: (NAN,) * 36)
    quality: float = NAN
    reason: str = 'MISSING_HEALTH'


def _check_health_contract(odometry, health, config, out):
    """按原顺序检查时间、来源和声明，仅在全部通过后填写健康与质量有效位。"""
    if not out.stamp_ns:
        # 0 同时代表无效或未提供的测量时间，不能参与有效状态转换。
        out.reason = 'INVALID_MEASUREMENT_STAMP'
        return False
    if health is None:
        # 保留默认 MISSING_HEALTH；不借用其他测量时刻的健康声明。
        return False
    out.sequence = getattr(health, 'sequence', 0)
    out.reset_counter = getattr(health, 'reset_counter', 0)
    if getattr(health, 'schema_version', 0) != 1 or out.sequence <= 0:
        # 序号和 reset 仍保留在诊断结果中，但错误协议不能授予任何测量有效位。
        out.reason = 'INVALID_HEALTH_VERSION_OR_SEQUENCE'
        return False
    if stamp_ns(health.stamp) != out.stamp_ns:
        # 两个消息必须描述同一测量时刻；接收得近不能代替严格配对。
        out.reason = 'HEALTH_STAMP_MISMATCH'
        return False
    for actual, expected in (
        (odometry.header.frame_id, config.input_frame_id),
        (odometry.child_frame_id, config.input_body_frame_id),
        (getattr(health, 'frame_id', ''), config.input_frame_id),
        (getattr(health, 'body_frame_id', ''), config.input_body_frame_id),
        (getattr(health, 'source_id', ''), config.source_id),
        (getattr(health, 'extrinsics_id', ''), config.extrinsics_id),
        (getattr(health, 'alignment_id', ''), config.alignment_id),
    ):
        if actual != expected:
            # 位置、速度、姿态必须属于同一来源及标定；不接受自动猜测或混合坐标。
            out.reason = 'SOURCE_FRAME_OR_CALIBRATION_ID_MISMATCH'
            return False
    # time_sync_valid 是上游对测量时钟的显式声明，不是这里测出的同步误差。
    # 本转换器不估算/修正时钟偏移；严格配对成功也不能替代消费者的源年龄和接收 TTL 检查。
    for name in (
        'reset_valid',
        'initialized',
        'tracking',
        'calibration_valid',
        'frame_alignment_valid',
        'time_sync_valid',
        'extrinsics_valid',
        'metric_scale_valid',
        'quality_valid',
    ):
        if not getattr(health, name, False):
            # 未确认尺度、时钟、跟踪或标定时拒绝，而不是让零值看起来像正常状态。
            out.reason = 'HEALTH_' + name.upper() + '_REQUIRED'
            return False
    quality = _finite((getattr(health, 'quality', NAN),), 1)
    if quality is None or not 0.0 <= quality[0] <= 1.0:
        # 这里检查质量的协议范围；实际控制可接受的质量阈值由后续消费者另行判断。
        out.reason = 'QUALITY_UNKNOWN_OR_OUT_OF_RANGE'
        return False
    # 到这里才确认健康合同；各物理量的有效位还需由下面对应的转换成功后设置。
    out.initialized = out.tracking = out.calibration_valid = True
    out.quality = quality[0]
    out.valid_fields = QUALITY
    return True


def convert_odometry(odometry, health, config):
    """转换一对实际测量；合同、坐标或数据缺失时保留无效位及 NaN。"""
    # 1. 配置是显式合同；默认结果不含任何已确认的测量，防止失败分支填入伪零值。
    config.validate()
    out = ConvertedState(source_id=config.source_id)
    # 在首次复制来源/坐标身份或扫描其内容之前拒绝；超限消息不获得任何测量有效位。
    identity_error = resource_budget.odometry_identity_error(odometry) or (
        resource_budget.health_identity_error(health)
    )
    if identity_error:
        out.reason = identity_error
        return out
    out.stamp_ns = stamp_ns(odometry.header.stamp)
    # 健康声明与测量必须严格配对；不满足合同就返回默认无效结果及首个拒绝原因。
    if not _check_health_contract(odometry, health, config, out):
        return out
    # 2. 建立坐标链：原始世界 → 约定 NED → 显式世界对齐。
    axis_world = ENU_TO_NED if config.world_convention == 'ENU' else IDENTITY
    axis_sensor = FLU_TO_FRD if config.body_convention == 'FLU' else IDENTITY
    world = _mul(_matrix(_quaternion(config.world_alignment_wxyz)), axis_world)
    body_sensor = _matrix(_quaternion(config.body_from_sensor_wxyz))
    position_sensor_body = config.sensor_position_body_frd
    lever_arm = any(x != 0.0 for x in position_sensor_body)
    raw_pose, raw_twist = odometry.pose.pose, odometry.twist.twist
    raw_q = raw_pose.orientation
    q = _quaternion(
        (
            getattr(raw_q, 'w', NAN),
            getattr(raw_q, 'x', NAN),
            getattr(raw_q, 'y', NAN),
            getattr(raw_q, 'z', NAN),
        )
    )
    orientation = None
    if getattr(health, 'attitude_valid', False) and q is not None:
        # 原四元数是 child → 原世界；再反向应用轴约定和安装旋转，得到机体 FRD → NED。
        orientation_sensor = _mul(_mul(world, _matrix(q)), _transpose(axis_sensor))
        orientation = _mul(orientation_sensor, _transpose(body_sensor))
        out.attitude = _matrix_quaternion(orientation)
        out.valid_fields |= ATTITUDE

    # 3. 位置单位为 m。传感器测量的是传感器原点，需要减去旋转到世界系的机体杆臂。
    position = _vector(raw_pose.position)
    if (
        getattr(health, 'position_valid', False)
        and position is not None
        and (not lever_arm or orientation is not None)
    ):
        # 零杆臂时不依赖姿态；非零杆臂时必须先有可信姿态，才能确定世界系偏移。
        p_sensor = tuple(
            a + b for a, b in zip(_rotate(world, position), config.world_translation_ned)
        )
        offset = _rotate(orientation, position_sensor_body) if lever_arm else (0.0, 0.0, 0.0)
        converted_position = _finite((a - b for a, b in zip(p_sensor, offset)), 3)
        if converted_position is not None:
            # 中间旋转或平移可能溢出；只有有限的派生结果才允许设置 POSITION 位。
            out.position = converted_position
            out.valid_fields |= POSITION

    # 4. 角速度单位为 rad/s，先统一传感器轴约定，再按安装外参转换为机体 FRD。
    omega = _vector(raw_twist.angular)
    if (
        getattr(health, 'angular_velocity_valid', False)
        and omega is not None
        and getattr(health, 'angular_velocity_source', '').strip()
    ):
        # 必须知道角速度的实际来源，不能用虚构的零角速度完成杆臂速度修正。
        converted_omega = _finite(_rotate(body_sensor, _rotate(axis_sensor, omega)), 3)
        if converted_omega is not None:
            out.angular_velocity = converted_omega
            out.angular_velocity_source = health.angular_velocity_source
            out.valid_fields |= ANGULAR_VELOCITY

    # 5. Odometry.twist 线速度位于 child 坐标系，单位 m/s，不是已经得到的世界速度。
    velocity = _vector(raw_twist.linear)
    if (
        getattr(health, 'velocity_valid', False)
        and velocity is not None
        and orientation is not None
        and (not lever_arm or out.valid_fields & ANGULAR_VELOCITY)
    ):
        # 使用原 child → 世界旋转，再应用世界对齐；数学运算顺序与原实现相同。
        v_sensor = _rotate(world, _rotate(_matrix(q), velocity))
        # 传感器安装偏离机体原点时，扣除 ω×r 带来的旋转线速度；缺角速度就不能置位。
        correction = (
            _rotate(orientation, _cross(out.angular_velocity, position_sensor_body))
            if lever_arm
            else (0.0, 0.0, 0.0)
        )
        converted_velocity = _finite((a - b for a, b in zip(v_sensor, correction)), 3)
        if converted_velocity is not None:
            out.velocity = converted_velocity
            out.valid_fields |= VELOCITY

    # 6. 协方差跨坐标和原点需要完整雅可比；本实现保留 NaN，不设置两个协方差有效位。
    out.reason = 'OK' if out.valid_fields & REQUIRED == REQUIRED else 'MISSING_REQUIRED_FIELDS'
    return out


# 配对缓存只认同一测量纳秒；接收时间只控制等待寿命，不做最近时间插值或跨测量健康借用。
class StampPairBuffer:
    """有容量上限的精确时间配对；重复包不能刷新等待时间或重复发布。"""

    def __init__(self, timeout=0.05, max_pending=32):
        if not math.isfinite(timeout) or timeout <= 0 or max_pending < 1:
            raise ValueError('positive finite pair timeout and queue size required')
        # timeout 是本机单调接收时间的秒数，不是上游测量时间戳的差。
        self.timeout, self.max_pending = timeout, max_pending
        self._odometry, self._health = OrderedDict(), OrderedDict()
        self._last_emitted = -1  # 已处理的最大测量时间，ns；清空待配对队列也不回退。

    def _finish(self, key, metadata):
        # 一旦完成或超时，移除两边并推进水位；同一测量以后不能借新健康声明复活。
        message, _ = self._odometry.pop(key)
        self._health.pop(key, None)
        self._last_emitted = max(self._last_emitted, key)
        return (message, metadata)

    def odometry(self, message, now):
        # Guarded 入口负责发布撤销；独立使用此低层缓存时也不能保存超预算半包。
        if resource_budget.odometry_identity_error(message):
            raise ValueError(resource_budget.RESOURCE_ERROR)
        # 先处理已经超时的等待项；晚到的另一半不能绕过配对窗口恢复旧样本。
        result = self.expire(now)
        key = stamp_ns(message.header.stamp)
        if key <= self._last_emitted or key in self._odometry:
            # 旧测量和重复半包都不刷新第一次接收时间。
            return result
        self._odometry[key] = (message, now)
        if key in self._health:
            # 之前的超时撤销必须排在新配对前，不能被本轮健康消息覆盖。
            result.append(self._finish(key, self._health[key][0]))
            return result
        if len(self._odometry) > self.max_pending:
            # 容量超限时将最早等待的测量明确作为缺失健康声明输出，不无限积压。
            result.append(self._finish(next(iter(self._odometry)), None))
        return result

    def health(self, message, now):
        if resource_budget.health_identity_error(message):
            raise ValueError(resource_budget.RESOURCE_ERROR)
        # 回调同样承担超时观察，不能假设定时器一定先于新消息执行。
        result = self.expire(now)
        key = stamp_ns(message.stamp)
        if key <= self._last_emitted or key in self._health:
            # 退役时间和重复健康声明不能延长窗口，reset 也不能改变这一配对水位。
            return result
        self._health[key] = (message, now)
        if key in self._odometry:
            result.append(self._finish(key, message))
            return result
        while len(self._health) > self.max_pending:
            # 健康声明本身不是测量；没有对应 Odometry 的旧声明只需淘汰。
            self._health.popitem(last=False)
        return result

    def expire(self, now):
        result = []
        # 使用列表副本遍历，允许处理过程中安全删除项，并保留原始到达顺序。
        for key, (_, received) in list(self._odometry.items()):
            if key <= self._last_emitted:
                # 更晚测量已经结束处理，此项不能再作为新测量输出。
                self._odometry.pop(key)
            elif now - received >= self.timeout:
                # 达到边界即超时；输出缺健康的撤销，不能等待晚到包覆盖这个事实。
                result.append(self._finish(key, None))
        for key, (_, received) in list(self._health.items()):
            if key <= self._last_emitted or now - received >= self.timeout:
                # 与测量队列使用同一窗口，避免过期声明为之后的原始数据背书。
                self._health.pop(key)
        return result

    def clear(self):
        # 只清待配对数据，保留已输出水位；来源恢复后仍不能重放旧测量。
        self._odometry.clear()
        self._health.clear()


class GuardedOdometryAdapter:
    """同时检查两个原始话题的发布者数量，再允许配对和转换。

    只看下游适配器发布者数量，无法发现它背后有多个原始来源。
    每次回调及无消息时的定时观察都要提供实际图数量；这里的准入只针对状态
    适配，不授予飞行权限，不生成目标速度或 PX4 动力指令。
    """

    def __init__(self, config, pair_timeout=0.05, max_pending=32):
        config.validate()
        self.config = config
        self.pairs = StampPairBuffer(pair_timeout, max_pending)
        self._counts = None
        self._sequence = 0
        self._last_stamp = 0
        self._last_reset = 0

    def _publish(self, value, ros_now_ns):
        # 每个转换或撤销事件都获得新的输出序号，上游健康序号另留诊断字段。
        self._sequence += 1
        value.raw_health_sequence = value.sequence
        value.sequence = self._sequence
        if value.valid_fields == 0:
            # 撤销事件标记的是观察时间，不是新的姿态测量；有效位为零、健康为假、数据为 NaN。
            value.stamp_ns = max(value.stamp_ns, self._last_stamp, ros_now_ns)
            value.reset_counter = self._last_reset
        else:
            # 包含已确认字段的结果沿用测量 reset；缺失健康的撤销不能虚构新估计周期。
            self._last_reset = value.reset_counter
        self._last_stamp = max(self._last_stamp, value.stamp_ns)
        return value

    def _authority(self, ros_now_ns, odometry_publishers, health_publishers):
        counts = (odometry_publishers, health_publishers)
        previous = self._counts
        self._counts = counts
        if counts != (1, 1):
            # 任一原始话题没有发布者或有竞争发布者时，撤销本地半包，禁止跨来源拼接。
            self.pairs.clear()
            if counts != previous:
                # 数量变化时发布一次明确撤销；相同异常数量下不重复制造状态事件。
                reason = 'RAW_SOURCE_COUNT_ODOMETRY_%s_HEALTH_%s' % counts
                value = ConvertedState(source_id=self.config.source_id, reason=reason)
                return False, [self._publish(value, ros_now_ns)]
            return False, []
        if previous != counts:
            # 图恢复为各一个来源后，必须等待恢复后的新配对，不能沿用旧半包。
            self.pairs.clear()
        return True, []

    def _converted(self, pairs, ros_now_ns):
        # 严格保留配对器给出的顺序：先前超时的撤销必须早于本次正常新状态。
        return [
            self._publish(convert_odometry(raw, health, self.config), ros_now_ns)
            for raw, health in pairs
        ]

    def _reject_resource_identity(self, now, ros_now_ns):
        """先输出已到期的撤销，再清半包并发本次资源拒绝；不保留或回显原始大字符串。

        now 是单调接收秒，ros_now_ns 是观察纳秒。每次拒绝取得新输出序号，不能因原始
        包的旧序号/旧时间而忽略本次撤销。清半包仍保留配对水位，不授予任何飞行权限。
        """
        expired = self._converted(self.pairs.expire(now), ros_now_ns)
        self.pairs.clear()
        invalid = ConvertedState(
            source_id=self.config.source_id, reason=resource_budget.RESOURCE_ERROR
        )
        return expired + [self._publish(invalid, ros_now_ns)]

    def odometry(self, message, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        if allowed:
            if resource_budget.odometry_identity_error(message):
                return self._reject_resource_identity(now, ros_now_ns)
            # 来源通过才允许修改配对缓存；新消息处理仍先观察旧项是否已经超时。
            return self._converted(self.pairs.odometry(message, now), ros_now_ns)
        return states

    def health(self, message, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        if allowed:
            if resource_budget.health_identity_error(message):
                return self._reject_resource_identity(now, ros_now_ns)
            # 健康消息只参与严格时间配对，不单独构造一个有效的飞机状态。
            return self._converted(self.pairs.health(message, now), ros_now_ns)
        return states

    def poll(self, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        if allowed:
            # 即使没有新消息，定时器仍须撤销超时配对，并观察原始来源数量变化。
            return self._converted(self.pairs.expire(now), ros_now_ns)
        return states


def _adapter_node_type():
    """延迟加载 ROS 并返回适配节点类；无参数，不创建节点、上下文或通信端点。

    可在纯转换模块导入后按需调用；返回类必须在 main 初始化 ROS 后实例化。
    将启动配置、端点接线与逐状态输出分开，但保留其原有执行先后顺序。
    """
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType
    from nav_msgs.msg import Odometry
    from jikl_fc01_system.msg import VisionHealth, VisionState

    class OdometryAdapterNode(Node):
        def __init__(self):
            """只构造 ROS 基类节点；成功返回后由 main 持有并负责 destroy_node。

            无输入参数或返回值。适配器配置和接线推迟到 configure_adapter，避免后续异常
            使 main 丢失已经创建的 Node。基类构造自己失败时，仍由 rclpy 管理其内部资源。
            """
            super().__init__('fc01_vision_odometry_adapter')

        def configure_adapter(self):
            """按时钟→标定→话题/QoS→输出→配对器→输入的原顺序完成接线。

            无输入参数或返回值；持有发布器、订阅、定时器和配对状态。main 必须先持有
            成功构造的节点；本方法失败也由 main 销毁，不在异常配置上运行回调。
            首个回调只能在 main 注册 executor 后运行，本节点不创建飞控控制端点。
            """
            self.configure_measurement_clock()
            self.load_conversion_config()
            input_topic, health_topic = self.input_topics()
            output_topic = self.startup_parameter('output_topic', '/fc01/vision/state')
            depth = self.startup_parameter('qos_depth', 5)
            if depth < 1 or depth > 1000:
                # 保留既有队列容量范围，限制 DDS 缓冲规模。
                raise ValueError('qos_depth must be in [1,1000]')

            # 只发布 FC01 实际状态消息，不创建 PX4 控制、解锁或动力设定值发布者。
            self.publisher = self.create_publisher(
                VisionState,
                output_topic,
                self.qos(depth, self.startup_parameter('output_reliable', False)),
            )
            self.adapter = GuardedOdometryAdapter(
                self.config,
                self.startup_parameter('pair_timeout', 0.05),
                self.startup_parameter('max_pending', 32),
            )
            self.input_topic, self.health_topic = input_topic, health_topic
            self.last_reason = ''
            self.connect_inputs(input_topic, health_topic, depth)
            self.get_logger().info(
                'Explicit V1 Odometry adapter; input='
                + input_topic
                + ' health='
                + health_topic
                + ' output='
                + output_topic
            )

        def startup_parameter(self, name, default):
            """声明一个启动只读参数并返回其值；name/default 原样交给 ROS，不做隐式转换。

            副作用是按构造调用顺序声明参数；必须在创建对应端点前完成，运行时不能续改标定或时钟。
            """
            return self.declare_parameter(
                name,
                default,
                descriptor=ParameterDescriptor(
                    read_only=True,
                    description='Startup-only adapter configuration; restart to change.',
                ),
            ).value

        def configure_measurement_clock(self):
            """拒绝模拟时钟并固定系统测量时钟描述符；无返回值，仅修改参数描述符。

            在读取标定和创建端点之前执行；不将单调等待时间解释成 ROS 测量时间。
            """
            if self.get_parameter('use_sim_time').value:
                # 当前接口只接受已约定的 ROS/系统测量时钟，不能自动改用模拟时钟。
                raise ValueError('V1 adapter requires agreed ROS/system measurement stamps')
            # use_sim_time 已由 ROS 声明；固定其描述符，避免运行中切换测量时钟域。
            self.set_descriptor(
                'use_sim_time',
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    read_only=True,
                    description='V1 adapter requires the agreed ROS/system time domain.',
                ),
            )

        def load_conversion_config(self):
            """按原字段次序读取显式坐标/外参并校验，成功后保存在 self.config；无返回值。

            米制 NED/FRD 和 wxyz 约定由 AdapterConfig 验证；缺项仍用 NaN 拒绝，不填充虚构标定。
            必须先完成时钟检查，失败时仍在创建任何数据端点之前抛出原异常。
            """
            # 没有通用的相机坐标或安装默认值；启动者必须明确填写接口合同。
            values = {
                name: self.startup_parameter(name, '')
                for name in (
                    'source_id',
                    'input_frame_id',
                    'input_body_frame_id',
                    'world_convention',
                    'body_convention',
                    'extrinsics_id',
                    'alignment_id',
                )
            }
            for name, length in (
                ('body_from_sensor_wxyz', 4),
                ('sensor_position_body_frd', 3),
                ('world_alignment_wxyz', 4),
                ('world_translation_ned', 3),
            ):
                # 未配置的旋转/平移保留 NaN，让校验拒绝，不能伪造单位旋转或零杆臂。
                values[name] = tuple(self.startup_parameter(name, [NAN] * length))
            self.config = AdapterConfig(**values)
            self.config.validate()

        def input_topics(self):
            """声明并返回 (Odometry, Health) 输入话题；只声明参数，缺一项则按原文本拒绝。

            此步骤在标定验证后、输出话题与 QoS 参数前执行，保持首个错误及声明顺序。
            """
            input_topic, health_topic = self.startup_parameter(
                'input_topic', ''
            ), self.startup_parameter('health_topic', '')
            if not input_topic or not health_topic:
                # 厂商话题名尚未通用约定，禁止自动猜测另一个传感器的数据入口。
                raise ValueError(
                    'explicit input_topic and health_topic are required; vendor topics are unknown'
                )
            return input_topic, health_topic

        def qos(self, depth, reliable):
            """按给定队列深度和可靠性构造 QoS；返回新配置，不修改节点或保存历史消息。

            depth 已先通过 [1,1000] 校验；VOLATILE 与可靠性选择保持原接口合同。
            """
            return QoSProfile(
                depth=depth,
                durability=DurabilityPolicy.VOLATILE,
                reliability=(
                    ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
                ),
            )

        def connect_inputs(self, input_topic, health_topic, depth):
            """按 Odometry→Health→超时定时器次序建立输入，保存三个句柄；无返回值。

            输入参数是已校验的话题和队列深度。self.adapter/publisher/话题状态必须先存在；
            回调逐次重新取单调秒、ROS 纳秒和来源数，不能缓存来源授权或改变撤销事件顺序。
            """
            # 每个回调都重新观察本机时间与两个原始来源数量，再把结果送入适配器。
            self.odom_subscription = self.create_subscription(
                Odometry,
                input_topic,
                lambda message: self.emit(self.adapter.odometry(message, *self.source_context())),
                self.qos(depth, self.startup_parameter('odometry_reliable', False)),
            )
            self.health_subscription = self.create_subscription(
                VisionHealth,
                health_topic,
                lambda message: self.emit(self.adapter.health(message, *self.source_context())),
                self.qos(depth, self.startup_parameter('health_reliable', True)),
            )
            # 无消息时仍观察配对超时和图冲突；定时周期保持原来的 min(10 ms, 配对窗口)。
            self.expiry_timer = self.create_timer(
                min(0.01, self.adapter.pairs.timeout),
                lambda: self.emit(self.adapter.poll(*self.source_context())),
            )

        def source_context(self):
            # 接收窗口使用单调秒数，测量/观察时间使用约定 ROS 纳秒；两者不能互相替代。
            return (
                time.monotonic(),
                max(0, self.get_clock().now().nanoseconds),
                self.count_publishers(self.input_topic),
                self.count_publishers(self.health_topic),
            )

        def state_message(self, converted):
            """将一个已转换状态映射成 VisionState；返回新消息，不发布、不修改配对或日志状态。

            测量/撤销观察时间仍按整数纳秒拆分；字段、有效位和 NaN 原样保留，不能在输出层补有效性。
            """
            message = VisionState()
            message.sequence = converted.sequence
            # 精确保留测量/撤销观察时间，按 ROS 的 sec + nanosec 格式拆分。
            message.stamp.sec = converted.stamp_ns // 1_000_000_000
            message.stamp.nanosec = converted.stamp_ns % 1_000_000_000
            # 消息身份与有效位原样传递，不在 ROS 输出层重新猜测测量可信性。
            for name in (
                'schema_version',
                'reset_counter',
                'frame_id',
                'body_frame_id',
                'source_id',
                'angular_velocity_source',
                'initialized',
                'tracking',
                'calibration_valid',
                'valid_fields',
                'quality',
            ):
                setattr(message, name, getattr(converted, name))
            for name in (
                'position',
                'velocity',
                'attitude',
                'angular_velocity',
                'pose_covariance',
                'twist_covariance',
            ):
                setattr(message, name, list(getattr(converted, name)))
            return message

        def report_reason_change(self, converted):
            """仅在原因变化时记录原日志文本，成功写日志后更新 last_reason；无返回值。

            必须在该状态消息发布后调用，日志节流不允许删去撤销消息；日志异常仍原样暴露。
            """
            if converted.reason != self.last_reason:
                # 日志只记录原因变化；状态消息仍逐项发布，不能用日志节流丢弃撤销事件。
                self.get_logger().info(
                    'state_quality='
                    + converted.reason
                    + ' canonical_sequence='
                    + str(converted.sequence)
                    + ' raw_health_sequence='
                    + str(converted.raw_health_sequence)
                )
                self.last_reason = converted.reason

        def emit(self, states):
            """按输入顺序发布每个转换/撤销状态，再记录原因变化；无返回值。

            states 由配对/来源监控产生；本函数只更新发布器与 last_reason，不授予飞行控制权限。
            消息映射→实际发布→原因日志的顺序与异常传播保持不变。
            """
            for converted in states:
                self.publisher.publish(self.state_message(converted))
                self.report_reason_change(converted)

    return OdometryAdapterNode


def _release_adapter_resources(actions, primary_error):
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


def main(args=None):
    """以可选 ROS 命令行 args 运行适配器；正常停止无返回值，非停止异常继续向上传播。

    仅此入口持有节点、executor 和临时信号处理器。SIGINT/SIGTERM 只请求停止，
    先结束当前回调，再移出/销毁节点、关闭 context、恢复调用者的处理器。
    100 ms 只是空闲 executor 等待上限，不是包含回调/清理的硬实时退出保证。
    """
    import signal

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    OdometryAdapterNode = _adapter_node_type()

    # ROS 的异步信号处理会先关闭 context，可能打断正在取消息/查询来源的回调。
    # Python 信号处理器仅置停止标志；不在 C 扩展取消息期间抛异常或关闭 context。
    stop_requested = False

    def request_stop(_signum, _frame):
        # 参数由 Python 提供；仅设置 main 拥有的停止标志，重复信号仍为幂等操作。
        # 不在异步信号中抛异常：待当前回调完整返回后，再退出 executor 循环并清理。
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    executor = None
    context_owned = False
    primary_error = None
    try:
        for signum in previous_handlers:
            signal.signal(signum, request_stop)
        # 仅清理由本入口从未初始化状态取得的默认 context；调用者已有的活动 context
        # 会让 rclpy.init 按原合同报错，不能因本次失败而把调用者的 context 关闭。
        context_owned = not rclpy.ok()
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = OdometryAdapterNode()
        node.configure_adapter()
        executor = rclpy.get_global_executor()
        executor.add_node(node)
        # 100 ms 只是空闲等待上限；已开始的回调和清理耗时不属于硬实时保证。
        # 已开始的回调先完整结束；与 rclpy.spin 相同，每次仍只执行 executor 的一项工作。
        while executor.context.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Python 停止信号或外部关闭属于正常停止；其他运行错误必须继续传播。
        pass
    except BaseException as error:
        primary_error = error
        raise
    finally:
        stop_requested = True
        # 清理仍由主线程依序完成；重复停止信号只置标志，不中断回调或释放动作。
        actions = []
        if executor is not None and node is not None:
            actions.append(lambda: executor.remove_node(node))
        if node is not None:
            actions.append(node.destroy_node)
        if context_owned:
            actions.append(lambda: rclpy.shutdown() if rclpy.ok() else None)
        for signum, handler in previous_handlers.items():
            actions.append(lambda signum=signum, handler=handler: signal.signal(signum, handler))
        _release_adapter_resources(actions, primary_error)


if __name__ == '__main__':
    main()
