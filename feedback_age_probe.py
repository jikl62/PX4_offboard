#!/usr/bin/env python3
# Humble被动反馈年龄观测：只订阅姿态/位置并读取status文件，保留调用方DDS域，不创建飞行发布者。
# 源时间微秒、RMW接收时间纳秒；RMW epoch未确认时host减RMW只是诊断，不能叫纯排队延迟。
"""Bounded passive Humble feedback-age observer; keeps the caller's DDS domain.

Source the same ROS 2 / px4_msgs environment as the controller, then run this
alongside it. This process subscribes to two feedback topics and reads status.json;
it never sends /fmu/in messages, movement targets, IPC commands, or serial traffic.
Standard rclpy parameter-event metadata is permitted; rosout/parameter services are
disabled. RMW timestamps are diagnostic values whose epoch needs confirmation.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import stat
import sys
import time

# 只观察/fmu/out；窗口30秒、报告5秒、样本上限限制长期诊断内存，不影响原控制回调。
TOPICS = {'attitude': '/fmu/out/vehicle_attitude',
          'local': '/fmu/out/vehicle_local_position'}
WINDOW_SECONDS = 30.
REPORT_SECONDS = 5.
SPIN_WAIT_SECONDS = .01
WINDOW_SAMPLE_CAP = 32768


# host/source同单位微秒才相减转秒；缺失源时间返回None，保留负年龄作为时钟偏差证据。
def age_seconds(host_us, source_us):
    return (host_us-source_us)*1e-6 if isinstance(source_us, int) and source_us > 0 else None


# 拒绝bool与非有限值后用于JSON诊断，不用零填补未知状态。
def number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


# 以单调秒中心化做OLS，输出年龄漂移斜率s/s与R²；统计趋势不能自行修正源时钟。
def slope(points):
    """OLS on centered monotonic seconds; preserves signed age and jump evidence."""
    if len(points) < 2:
        return {'samples': len(points), 'span_s': 0., 'slope_s_per_s': None, 'r_squared': None}
    origin = points[0][0]
    xs = [x-origin for x, _ in points]
    ys = [y for _, y in points]
    xbar, ybar = sum(xs)/len(xs), sum(ys)/len(ys)
    xx = sum((x-xbar)**2 for x in xs)
    yy = sum((y-ybar)**2 for y in ys)
    xy = sum((x-xbar)*(y-ybar) for x, y in zip(xs, ys))
    return {'samples': len(points), 'span_s': xs[-1],
            'slope_s_per_s': xy/xx if xx > 0 else None,
            'r_squared': min(1., max(0., xy*xy/(xx*yy))) if xx > 0 and yy > 0 else None}


# 只统计相邻时间戳推进/重复/回退/缺失，不替控制器决定reset或恢复。
class StampProgress:
    """Adjacent-observation counters only; this observer makes no reset decision."""
    def __init__(self):
        self.previous = None
        self.counts = dict(first=0, advanced=0, duplicate=0, regressed=0, missing=0)

    # 缺失时间不覆盖上一有效水位；下一帧仍与最后有效源时间比较。
    def observe(self, stamp):
        if not isinstance(stamp, int) or stamp <= 0:
            relation = 'missing'
        elif self.previous is None:
            relation = 'first'
        else:
            relation = 'advanced' if stamp > self.previous else 'duplicate' if stamp == self.previous else 'regressed'
        self.counts[relation] += 1
        if relation != 'missing':
            self.previous = stamp
        return relation


# 每话题独立计数、最后观测及有界时间窗；包络与样本分别跟踪，防止新包掩盖旧样本。
class TopicObservation:
    def __init__(self):
        self.callbacks = self.reported_callbacks = 0
        self.envelope_progress, self.sample_progress = StampProgress(), StampProgress()
        self.last = None
        self.window = deque()
        self.window_drops = 0

    # 保留原RMW元数据，记录实际回调wall/mono；按时间窗和数量上限淘汰诊断样本。
    def observe(self, envelope, sample, info, host_us, mono):
        self.callbacks += 1
        received_ns = info.get('received_timestamp') if isinstance(info, dict) else None
        received_available = isinstance(received_ns, int) and received_ns > 0
        envelope_age, sample_age = age_seconds(host_us, envelope), age_seconds(host_us, sample)
        rmw_age = (host_us*1000-received_ns)*1e-9 if received_available else None
        self.last = dict(envelope_us=envelope, sample_us=sample, callback_host_us=host_us,
                         callback_monotonic_s=mono, callback_envelope_age_s=envelope_age,
                         callback_sample_age_s=sample_age,
                         envelope_sample_gap_s=(envelope-sample)*1e-6 if envelope and sample else None,
                         rmw_received_timestamp_ns=received_ns, rmw_received_available=received_available,
                         rmw_to_callback_s=rmw_age,
                         envelope_relation=self.envelope_progress.observe(envelope),
                         sample_relation=self.sample_progress.observe(sample))
        self.window.append((mono, envelope_age, sample_age, rmw_age))
        while self.window and self.window[0][0] < mono-WINDOW_SECONDS:
            self.window.popleft()
        if len(self.window) > WINDOW_SAMPLE_CAP:
            self.window.popleft()
            self.window_drops += 1

    # 报告时重新计算最后源年龄与receipt age，不用最近一条回调中的旧年龄冒充当前。
    def report(self, host_us, mono, interval_s):
        while self.window and self.window[0][0] < mono-WINDOW_SECONDS:
            self.window.popleft()
        delta = self.callbacks-self.reported_callbacks
        self.reported_callbacks = self.callbacks
        result = dict(callbacks_total=self.callbacks, callbacks_interval=delta,
                      callback_rate_hz=delta/interval_s if interval_s > 0 else None,
                      envelope_progress=dict(self.envelope_progress.counts),
                      sample_progress=dict(self.sample_progress.counts), last=self.last,
                      window_s=WINDOW_SECONDS, window_sample_cap_drops_total=self.window_drops)
        for index, label in ((1, 'envelope_age'), (2, 'sample_age'), (3, 'rmw_to_callback')):
            result[label+'_trend'] = slope([(row[0], row[index]) for row in self.window if row[index] is not None])
        result.update(envelope_age_at_report_s=age_seconds(host_us, self.last['envelope_us']) if self.last else None,
                      sample_age_at_report_s=age_seconds(host_us, self.last['sample_us']) if self.last else None,
                      receipt_age_s=mono-self.last['callback_monotonic_s'] if self.last else None)
        return result


# 读取最多1MiB普通快照，与探针的不同采样时刻对照；不连接控制socket或刷新任务租约。
def read_controller_status(path, host_us, probe_topics):
    """Read one bounded local snapshot; never contacts the supervisor/controller."""
    result = {'path': str(path), 'available': False,
              'comparison_note': 'Different callbacks/snapshot instants; timestamp gaps alone are not transport latency.'}
    try:
        if not stat.S_ISREG(path.stat().st_mode):
            raise ValueError('status path is not a regular file')
        with path.open('rb') as stream:
            raw = stream.read(1024*1024+1)
        if len(raw) > 1024*1024:
            raise ValueError('status exceeds 1 MiB')
        row = json.loads(raw)
        if not isinstance(row, dict):
            raise ValueError('status root is not an object')
        state = row.get('state', row)
        state = state if isinstance(state, dict) else {}
        result.update(available=True, status=row.get('status'), feedback_current=row.get('feedback_current'),
                      boot_id=row.get('boot_id'), controller_pid=row.get('controller_pid'),
                      state_controller_pid=row.get('state_controller_pid'),
                      updated_unix=number(row.get('updated_unix')),
                      state_age_seconds=number(row.get('state_age_seconds')),
                      diagnostic_host_us=number(state.get('diagnostic_host_us')),
                      diagnostic_monotonic_s=number(state.get('diagnostic_monotonic_s')),
                      receive_policy=state.get('receive_policy'), topics={})
        result['status_wall_age_s'] = host_us*1e-6-result['updated_unix'] if result['updated_unix'] is not None else None
        timing = state.get('source_timing', {})
        timing = timing if isinstance(timing, dict) else {}
        for key, probe in probe_topics.items():
            source = timing.get(key, {})
            source = source if isinstance(source, dict) else {}
            envelope, sample = source.get('envelope_us'), source.get('sample_us')
            last = probe.get('last') or {}
            probe_stamp = last.get('envelope_us')
            result['topics'][key] = dict(envelope_us=envelope, sample_us=sample,
                envelope_age_at_controller_snapshot_s=number(source.get('envelope_age_s', state.get(key+'_source_age_s'))),
                sample_age_at_controller_snapshot_s=number(source.get('sample_age_s', state.get(key+'_sample_age_s'))),
                callback_envelope_age_s=number(source.get('callback_envelope_age_s')),
                rmw_to_callback_s=number(source.get('rmw_to_callback_s')),
                envelope_age_at_probe_report_s=age_seconds(host_us, envelope),
                probe_minus_controller_envelope_s=(probe_stamp-envelope)*1e-6 if probe_stamp and envelope else None,
                same_envelope=probe_stamp == envelope if probe_stamp and envelope else None)
    except (OSError, ValueError, TypeError) as exc:
        result['error'] = str(exc)
    return result


# 延迟导入Humble执行器；定制take保留真实RMW metadata，只适配已检查Humble API。
def ros_classes():
    from rclpy.executors import SingleThreadedExecutor

    class MetadataExecutor(SingleThreadedExecutor):
        # Humble's stock executor discards tuple[1] from take_message. Preserve
        # its original RMW info; no fabricated receive timestamp. These two small
        # hooks are specific to the Humble executor API and tested with 3.3.21.
        # https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/subscription.cpp#L124
        # 持subscription handle期间执行真实take，不能伪造receive时间。
        def _take_subscription(self, subscription):
            with subscription.handle:
                return subscription.handle.take_message(subscription.msg_type, subscription.raw)

        # 只在实际取到消息时把payload及同条info交给回调，保持两者配对。
        async def _execute_subscription(self, subscription, taken):
            if taken is not None:
                subscription.callback(taken[0], taken[1])

    return MetadataExecutor


# 显式限制观测5～180秒且必须有限，不允许NaN/Inf造成无界运行。
def duration_arg(value):
    seconds = float(value)
    if not math.isfinite(seconds) or not 5 <= seconds <= 180:
        raise argparse.ArgumentTypeError('duration must be between 5 and 180 seconds')
    return seconds


# 校验环境/新输出路径，创建仅订阅节点；单调期限结束后输出final，再依次回收执行器/节点/文件。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=duration_arg, default=180., help='seconds, 5..180; default 180')
    parser.add_argument('--status', type=Path, default=Path.home()/'FC01_navigation/status.json', help='read-only controller status path')
    parser.add_argument('--output', type=Path, help='also create a new JSONL file (must not already exist)')
    args = parser.parse_args(argv)
    try:
        import rclpy
        import px4_msgs
        from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        parser.error('source ROS 2 Humble and the controller px4_msgs workspace first: '+str(exc))
    output = None
    if args.output:
        try:
            output = args.output.expanduser().open('x', encoding='utf8')
        except OSError as exc:
            parser.error('cannot create output: '+str(exc))

    # JSON禁止NaN并同步写可选新文件；诊断输出不产生飞行动作。
    def emit(row):
        line = json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        print(line, flush=True)
        if output:
            output.write(line+'\n')
            output.flush()

    node = executor = None
    observations = {key: TopicObservation() for key in TOPICS}
    start_mono, start_host = time.monotonic(), time.time_ns()//1000
    last_report = start_mono
    spin_calls = 0
    interrupted = False
    try:
        rclpy.init(args=[])
        node = Node('fc01_feedback_age_probe', enable_rosout=False, start_parameter_services=False,
                    use_global_arguments=False)
        executor = ros_classes()()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, cls in (('attitude', VehicleAttitude), ('local', VehicleLocalPosition)):
            # 用默认参数冻结话题key；同一回调采集wall与mono，不能混用不同消息的时间。
            def receive(message, info, key=key):
                host, mono = time.time_ns()//1000, time.monotonic()
                observations[key].observe(getattr(message, 'timestamp', None),
                                          getattr(message, 'timestamp_sample', None), info, host, mono)
            node.create_subscription(cls, TOPICS[key], receive, qos)
        executor.add_node(node)
        emit(dict(kind='feedback_age_probe_start', duration_s=args.duration,
                  ros_domain_id=node.context.get_domain_id(), ros_domain_env=os.environ.get('ROS_DOMAIN_ID'),
                  rmw_implementation=rclpy.get_rmw_implementation_identifier(), px4_msgs_path=px4_msgs.__file__,
                  spin_wait_limit_s=SPIN_WAIT_SECONDS, report_interval_s=REPORT_SECONDS,
                  window_s=WINDOW_SECONDS, qos='BEST_EFFORT_KEEP_LAST_10',
                  control_publishers=False, standard_parameter_events_metadata=True,
                  rmw_timestamp_interpretation='UNVERIFIED_EPOCH_DIAGNOSTIC_HOST_WALL_MINUS_RMW; not pure queue time'))

        # 保留host相对steady锚误差以暴露墙钟跳变；状态对照只表示不同快照之间的差。
        def report(final=False):
            nonlocal last_report
            host, mono = time.time_ns()//1000, time.monotonic()
            topics = {key: value.report(host, mono, mono-last_report) for key, value in observations.items()}
            for key, value in topics.items():
                value['topic'] = TOPICS[key]
                value['publishers_observed'] = node.count_publishers(TOPICS[key])
            emit(dict(kind='feedback_age_probe_final' if final else 'feedback_age_probe',
                      host_us=host, monotonic_s=mono, elapsed_s=mono-start_mono,
                      host_vs_steady_anchor_error_s=(host-start_host)*1e-6-(mono-start_mono),
                      spin_calls_total=spin_calls, interrupted=interrupted, diagnostic_only=True,
                      topics=topics, controller_status=read_controller_status(args.status.expanduser(), host, topics)))
            last_report = mono

        next_report, deadline = start_mono+REPORT_SECONDS, start_mono+args.duration
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=max(0., min(SPIN_WAIT_SECONDS, next_report-time.monotonic(), deadline-time.monotonic())))
                spin_calls += 1
                now = time.monotonic()
                if now >= next_report and now < deadline:
                    report()
                    next_report += REPORT_SECONDS
        except KeyboardInterrupt:
            interrupted = True
        report(final=True)
    # 无论结束/中断都关闭自己的ROS资源和文件；不触及真实控制器或Agent。
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if output:
            output.close()
    return 130 if interrupted else 0


if __name__ == '__main__':
    sys.exit(main())
