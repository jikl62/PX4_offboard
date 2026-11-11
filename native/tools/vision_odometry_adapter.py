#!/usr/bin/env python3
"""Explicit Odometry + exact-stamp VisionHealth -> FC01 V1 state.

The conversion and bounded pair buffer are ROS-independent. Importing this module
never creates a node, opens hardware, or publishes. Covariances are deliberately
unavailable until their full frame/origin Jacobians have an implementation.
"""
from collections import OrderedDict
from dataclasses import dataclass, field
import math
import time

POSITION, VELOCITY, ATTITUDE, ANGULAR_VELOCITY = 1, 2, 4, 8
POSE_COVARIANCE, TWIST_COVARIANCE, QUALITY = 16, 32, 64
REQUIRED = POSITION | VELOCITY | ATTITUDE | QUALITY
NAN = math.nan
IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
ENU_TO_NED = ((0., 1., 0.), (1., 0., 0.), (0., 0., -1.))
FLU_TO_FRD = ((1., 0., 0.), (0., -1., 0.), (0., 0., -1.))


def stamp_ns(stamp):
    sec, ns = getattr(stamp, 'sec', -1), getattr(stamp, 'nanosec', -1)
    if not isinstance(sec, int) or not isinstance(ns, int) or sec < 0 or not 0 <= ns < 1_000_000_000:
        return 0
    return sec * 1_000_000_000 + ns


def _finite(values, count):
    try:
        result = tuple(float(x) for x in values)
        return result if len(result) == count and all(math.isfinite(x) for x in result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _quaternion(values):
    q = _finite(values, 4)
    if q is None:
        return None
    norm = math.sqrt(sum(x*x for x in q))
    return tuple(x/norm for x in q) if norm and abs(norm - 1.) <= .05 else None


def _matrix(q):
    w, x, y, z = q
    return ((1.-2.*(y*y+z*z), 2.*(x*y-z*w), 2.*(x*z+y*w)),
            (2.*(x*y+z*w), 1.-2.*(x*x+z*z), 2.*(y*z-x*w)),
            (2.*(x*z-y*w), 2.*(y*z+x*w), 1.-2.*(x*x+y*y)))


def _transpose(m):
    return tuple(zip(*m))


def _mul(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _rotate(m, v):
    return tuple(sum(m[i][k]*v[k] for k in range(3)) for i in range(3))


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _matrix_quaternion(m):
    # Select the largest squared component to remain stable at 180 degrees.
    candidates = (1.+m[0][0]+m[1][1]+m[2][2], 1.+m[0][0]-m[1][1]-m[2][2],
                  1.-m[0][0]+m[1][1]-m[2][2], 1.-m[0][0]-m[1][1]+m[2][2])
    branch = max(range(4), key=lambda i: candidates[i])
    s = 2.*math.sqrt(max(0., candidates[branch]))
    if branch == 0:
        q = (s/4., (m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s)
    elif branch == 1:
        q = ((m[2][1]-m[1][2])/s, s/4., (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s)
    elif branch == 2:
        q = ((m[0][2]-m[2][0])/s, (m[0][1]+m[1][0])/s, s/4., (m[1][2]+m[2][1])/s)
    else:
        q = ((m[1][0]-m[0][1])/s, (m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s, s/4.)
    norm = math.sqrt(sum(x*x for x in q))
    return tuple(x/norm * (-1. if q[0] < 0. else 1.) for x in q)


def _vector(message):
    return _finite((getattr(message, 'x', NAN), getattr(message, 'y', NAN),
                    getattr(message, 'z', NAN)), 3)


@dataclass(frozen=True)
class AdapterConfig:
    source_id: str
    input_frame_id: str
    input_body_frame_id: str
    world_convention: str
    body_convention: str
    # R_B_S maps sensor FRD into body FRD. r_B_S is sensor origin in body FRD.
    body_from_sensor_wxyz: tuple
    sensor_position_body_frd: tuple
    # Applied after conventional ENU->NED; explicitly establishes navigation alignment.
    world_alignment_wxyz: tuple
    world_translation_ned: tuple
    extrinsics_id: str
    alignment_id: str

    def validate(self):
        for name in ('source_id', 'input_frame_id', 'input_body_frame_id', 'extrinsics_id', 'alignment_id'):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError('explicit ' + name + ' is required')
        if self.world_convention not in ('NED', 'ENU') or self.body_convention not in ('FRD', 'FLU'):
            raise ValueError('explicit world ENU/NED and body FLU/FRD conventions are required')
        if _quaternion(self.body_from_sensor_wxyz) is None or _quaternion(self.world_alignment_wxyz) is None:
            raise ValueError('explicit finite unit extrinsic/alignment wxyz quaternions are required')
        if _finite(self.sensor_position_body_frd, 3) is None or _finite(self.world_translation_ned, 3) is None:
            raise ValueError('explicit finite extrinsic/alignment translations are required')


@dataclass
class ConvertedState:
    schema_version: int = 1
    sequence: int = 0
    # Diagnostic only: wire sequence belongs to the adapter, not to this source.
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
    valid_fields: int = 0
    position: tuple = field(default_factory=lambda: (NAN,) * 3)
    velocity: tuple = field(default_factory=lambda: (NAN,) * 3)
    attitude: tuple = field(default_factory=lambda: (NAN,) * 4)
    angular_velocity: tuple = field(default_factory=lambda: (NAN,) * 3)
    pose_covariance: tuple = field(default_factory=lambda: (NAN,) * 36)
    twist_covariance: tuple = field(default_factory=lambda: (NAN,) * 36)
    quality: float = NAN
    reason: str = 'MISSING_HEALTH'


def convert_odometry(odometry, health, config):
    """Convert one exact pair. Missing confirmations revoke validity, never fill truth."""
    config.validate()
    out = ConvertedState(source_id=config.source_id)
    out.stamp_ns = stamp_ns(odometry.header.stamp)
    if not out.stamp_ns:
        out.reason = 'INVALID_MEASUREMENT_STAMP'
        return out
    if health is None:
        return out
    out.sequence = getattr(health, 'sequence', 0)
    out.reset_counter = getattr(health, 'reset_counter', 0)
    if getattr(health, 'schema_version', 0) != 1 or out.sequence <= 0:
        out.reason = 'INVALID_HEALTH_VERSION_OR_SEQUENCE'
        return out
    if stamp_ns(health.stamp) != out.stamp_ns:
        out.reason = 'HEALTH_STAMP_MISMATCH'
        return out
    for actual, expected in ((odometry.header.frame_id, config.input_frame_id),
                             (odometry.child_frame_id, config.input_body_frame_id),
                             (getattr(health, 'frame_id', ''), config.input_frame_id),
                             (getattr(health, 'body_frame_id', ''), config.input_body_frame_id),
                             (getattr(health, 'source_id', ''), config.source_id),
                             (getattr(health, 'extrinsics_id', ''), config.extrinsics_id),
                             (getattr(health, 'alignment_id', ''), config.alignment_id)):
        if actual != expected:
            out.reason = 'SOURCE_FRAME_OR_CALIBRATION_ID_MISMATCH'
            return out
    for name in ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                 'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                 'metric_scale_valid', 'quality_valid'):
        if not getattr(health, name, False):
            out.reason = 'HEALTH_' + name.upper() + '_REQUIRED'
            return out
    quality = _finite((getattr(health, 'quality', NAN),), 1)
    if quality is None or not 0. <= quality[0] <= 1.:
        out.reason = 'QUALITY_UNKNOWN_OR_OUT_OF_RANGE'
        return out
    out.initialized = out.tracking = out.calibration_valid = True
    out.quality = quality[0]
    out.valid_fields = QUALITY
    axis_world = ENU_TO_NED if config.world_convention == 'ENU' else IDENTITY
    axis_sensor = FLU_TO_FRD if config.body_convention == 'FLU' else IDENTITY
    world = _mul(_matrix(_quaternion(config.world_alignment_wxyz)), axis_world)
    body_sensor = _matrix(_quaternion(config.body_from_sensor_wxyz))
    position_sensor_body = config.sensor_position_body_frd
    lever_arm = any(x != 0. for x in position_sensor_body)
    raw_pose, raw_twist = odometry.pose.pose, odometry.twist.twist
    raw_q = raw_pose.orientation
    q = _quaternion((getattr(raw_q, 'w', NAN), getattr(raw_q, 'x', NAN),
                     getattr(raw_q, 'y', NAN), getattr(raw_q, 'z', NAN)))
    orientation = None
    if getattr(health, 'attitude_valid', False) and q is not None:
        orientation_sensor = _mul(_mul(world, _matrix(q)), _transpose(axis_sensor))
        orientation = _mul(orientation_sensor, _transpose(body_sensor))
        out.attitude = _matrix_quaternion(orientation)
        out.valid_fields |= ATTITUDE
    position = _vector(raw_pose.position)
    if getattr(health, 'position_valid', False) and position is not None and (not lever_arm or orientation is not None):
        p_sensor = tuple(a+b for a, b in zip(_rotate(world, position), config.world_translation_ned))
        offset = _rotate(orientation, position_sensor_body) if lever_arm else (0., 0., 0.)
        converted_position = _finite((a-b for a, b in zip(p_sensor, offset)), 3)
        if converted_position is not None:
            out.position = converted_position
            out.valid_fields |= POSITION
    omega = _vector(raw_twist.angular)
    if (getattr(health, 'angular_velocity_valid', False) and omega is not None
            and getattr(health, 'angular_velocity_source', '').strip()):
        converted_omega = _finite(_rotate(body_sensor, _rotate(axis_sensor, omega)), 3)
        if converted_omega is not None:
            out.angular_velocity = converted_omega
            out.angular_velocity_source = health.angular_velocity_source
            out.valid_fields |= ANGULAR_VELOCITY
    velocity = _vector(raw_twist.linear)
    if (getattr(health, 'velocity_valid', False) and velocity is not None and orientation is not None
            and (not lever_arm or out.valid_fields & ANGULAR_VELOCITY)):
        # Odometry twist is in its child frame, NOT its world frame.
        v_sensor = _rotate(world, _rotate(_matrix(q), velocity))
        correction = _rotate(orientation, _cross(out.angular_velocity, position_sensor_body)) if lever_arm else (0., 0., 0.)
        converted_velocity = _finite((a-b for a, b in zip(v_sensor, correction)), 3)
        if converted_velocity is not None:
            out.velocity = converted_velocity
            out.valid_fields |= VELOCITY
    # Both covariance bits stay clear: changing frame/origin requires full Jacobians.
    out.reason = 'OK' if out.valid_fields & REQUIRED == REQUIRED else 'MISSING_REQUIRED_FIELDS'
    return out


class StampPairBuffer:
    """Bounded exact-stamp join. A duplicate never refreshes or republishes a sample."""
    def __init__(self, timeout=.05, max_pending=32):
        if not math.isfinite(timeout) or timeout <= 0 or max_pending < 1:
            raise ValueError('positive finite pair timeout and queue size required')
        self.timeout, self.max_pending = timeout, max_pending
        self._odometry, self._health = OrderedDict(), OrderedDict()
        self._last_emitted = -1

    def _finish(self, key, metadata):
        message, _ = self._odometry.pop(key)
        self._health.pop(key, None)
        self._last_emitted = max(self._last_emitted, key)
        return (message, metadata)

    def odometry(self, message, now):
        # 先处理已经超时的等待项；晚到的另一半不能绕过配对窗口恢复旧样本。
        result = self.expire(now)
        key = stamp_ns(message.header.stamp)
        if key <= self._last_emitted or key in self._odometry:
            return result
        self._odometry[key] = (message, now)
        if key in self._health:
            # 之前的超时撤销必须排在新配对前，不能被本轮健康消息覆盖。
            result.append(self._finish(key, self._health[key][0]))
            return result
        if len(self._odometry) > self.max_pending:
            result.append(self._finish(next(iter(self._odometry)), None))
        return result

    def health(self, message, now):
        # 回调同样承担超时观察，不能假设定时器一定先于新消息执行。
        result = self.expire(now)
        key = stamp_ns(message.stamp)
        if key <= self._last_emitted or key in self._health:
            return result
        self._health[key] = (message, now)
        if key in self._odometry:
            result.append(self._finish(key, message))
            return result
        while len(self._health) > self.max_pending:
            self._health.popitem(last=False)
        return result

    def expire(self, now):
        result = []
        for key, (_, received) in list(self._odometry.items()):
            if key <= self._last_emitted:
                self._odometry.pop(key)
            elif now - received >= self.timeout:
                result.append(self._finish(key, None))
        for key, (_, received) in list(self._health.items()):
            if key <= self._last_emitted or now - received >= self.timeout:
                self._health.pop(key)
        return result

    def clear(self):
        # Keep the emitted watermark: graph recovery cannot replay an old pair.
        self._odometry.clear()
        self._health.clear()


class GuardedOdometryAdapter:
    """Join/conversion authority using BOTH actual upstream graph counts.

    Canonical publisher count alone cannot expose multiple publishers hidden
    behind an adapter. Callers must supply current counts on every callback and
    periodically even when neither input produces callbacks.
    """
    def __init__(self, config, pair_timeout=.05, max_pending=32):
        config.validate()
        self.config = config
        self.pairs = StampPairBuffer(pair_timeout, max_pending)
        self._counts = None
        self._sequence = 0
        self._last_stamp = 0
        self._last_reset = 0

    def _publish(self, value, ros_now_ns):
        self._sequence += 1
        value.raw_health_sequence = value.sequence
        value.sequence = self._sequence
        if value.valid_fields == 0:
            # A revocation event has an observation time, not a newly measured
            # pose. Mask=0, false health flags and NaNs keep that distinction.
            value.stamp_ns = max(value.stamp_ns, self._last_stamp, ros_now_ns)
            value.reset_counter = self._last_reset
        else:
            self._last_reset = value.reset_counter
        self._last_stamp = max(self._last_stamp, value.stamp_ns)
        return value

    def _authority(self, ros_now_ns, odometry_publishers, health_publishers):
        counts = (odometry_publishers, health_publishers)
        previous = self._counts
        self._counts = counts
        if counts != (1, 1):
            self.pairs.clear()
            if counts != previous:
                reason = 'RAW_SOURCE_COUNT_ODOMETRY_%s_HEALTH_%s' % counts
                value = ConvertedState(source_id=self.config.source_id, reason=reason)
                return False, [self._publish(value, ros_now_ns)]
            return False, []
        if previous != counts:
            self.pairs.clear()
        return True, []

    def _converted(self, pairs, ros_now_ns):
        return [self._publish(convert_odometry(raw, health, self.config), ros_now_ns)
                for raw, health in pairs]

    def odometry(self, message, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        return self._converted(self.pairs.odometry(message, now), ros_now_ns) if allowed else states

    def health(self, message, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        return self._converted(self.pairs.health(message, now), ros_now_ns) if allowed else states

    def poll(self, now, ros_now_ns, odometry_publishers, health_publishers):
        allowed, states = self._authority(ros_now_ns, odometry_publishers, health_publishers)
        return self._converted(self.pairs.expire(now), ros_now_ns) if allowed else states


def main(args=None):
    # ROS is a runtime-only dependency: conversion unit tests never need a graph.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType
    from nav_msgs.msg import Odometry
    from jikl_fc01_system.msg import VisionHealth, VisionState

    class OdometryAdapterNode(Node):
        def __init__(self):
            super().__init__('fc01_vision_odometry_adapter')
            def parameter(name, default):
                # 配置只在启动时应用；运行期明确拒绝修改，避免显示值与实际转换不同。
                return self.declare_parameter(name, default, descriptor=ParameterDescriptor(
                    read_only=True, description='Startup-only adapter configuration; restart to change.')).value
            if self.get_parameter('use_sim_time').value:
                raise ValueError('V1 adapter requires agreed ROS/system measurement stamps')
            # use_sim_time 已由 ROS 声明；固定其描述符，避免运行中切换测量时钟域。
            self.set_descriptor('use_sim_time', ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL, read_only=True,
                description='V1 adapter requires the agreed ROS/system time domain.'))
            values = {name: parameter(name, '') for name in
                      ('source_id', 'input_frame_id', 'input_body_frame_id', 'world_convention',
                       'body_convention', 'extrinsics_id', 'alignment_id')}
            for name, length in (('body_from_sensor_wxyz', 4), ('sensor_position_body_frd', 3),
                                 ('world_alignment_wxyz', 4), ('world_translation_ned', 3)):
                values[name] = tuple(parameter(name, [NAN] * length))
            self.config = AdapterConfig(**values)
            self.config.validate()
            input_topic, health_topic = parameter('input_topic', ''), parameter('health_topic', '')
            if not input_topic or not health_topic:
                raise ValueError('explicit input_topic and health_topic are required; vendor topics are unknown')
            output_topic = parameter('output_topic', '/fc01/vision/state')
            depth = parameter('qos_depth', 5)
            if depth < 1 or depth > 1000:
                raise ValueError('qos_depth must be in [1,1000]')
            def qos(reliable):
                return QoSProfile(depth=depth, durability=DurabilityPolicy.VOLATILE,
                                  reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
            self.publisher = self.create_publisher(VisionState, output_topic, qos(parameter('output_reliable', False)))
            self.adapter = GuardedOdometryAdapter(self.config, parameter('pair_timeout', .05),
                                                 parameter('max_pending', 32))
            self.input_topic, self.health_topic = input_topic, health_topic
            self.last_reason = ''
            self.odom_subscription = self.create_subscription(
                Odometry, input_topic, lambda message: self.emit(self.adapter.odometry(message, *self.source_context())),
                qos(parameter('odometry_reliable', False)))
            self.health_subscription = self.create_subscription(
                VisionHealth, health_topic, lambda message: self.emit(self.adapter.health(message, *self.source_context())),
                qos(parameter('health_reliable', True)))
            self.expiry_timer = self.create_timer(min(.01, self.adapter.pairs.timeout),
                                                 lambda: self.emit(self.adapter.poll(*self.source_context())))
            self.get_logger().info('Explicit V1 Odometry adapter; input=' + input_topic +
                                   ' health=' + health_topic + ' output=' + output_topic)

        def source_context(self):
            return (time.monotonic(), max(0, self.get_clock().now().nanoseconds),
                    self.count_publishers(self.input_topic), self.count_publishers(self.health_topic))

        def emit(self, states):
            for converted in states:
                message = VisionState()
                message.sequence = converted.sequence
                message.stamp.sec = converted.stamp_ns // 1_000_000_000
                message.stamp.nanosec = converted.stamp_ns % 1_000_000_000
                for name in ('schema_version', 'reset_counter', 'frame_id', 'body_frame_id', 'source_id',
                             'angular_velocity_source', 'initialized', 'tracking', 'calibration_valid',
                             'valid_fields', 'quality'):
                    setattr(message, name, getattr(converted, name))
                for name in ('position', 'velocity', 'attitude', 'angular_velocity',
                             'pose_covariance', 'twist_covariance'):
                    setattr(message, name, list(getattr(converted, name)))
                self.publisher.publish(message)
                if converted.reason != self.last_reason:
                    self.get_logger().info('state_quality=' + converted.reason +
                                           ' canonical_sequence=' + str(converted.sequence) +
                                           ' raw_health_sequence=' + str(converted.raw_health_sequence))
                    self.last_reason = converted.reason

    rclpy.init(args=args)
    node = None
    try:
        node = OdometryAdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT 或外部关闭属于正常停止；上下文可能已被 ROS 的信号处理器关闭。
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # 只关闭仍存活的上下文，避免正常退出被第二次 shutdown 的异常覆盖。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
