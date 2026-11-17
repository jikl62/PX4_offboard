"""对任一适配器源码运行相同假 ROS 事件流，输出参数/接线/消息/日志的严格有序轨迹。"""

import importlib.util
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
from unittest.mock import patch

from test_stage2_vision_shutdown import FakeRos
from test_vision_adapter import health, odometry, stamp


def canonical(value):
    """将记录转成可比较 JSON，浮点使用 hex 精确保留数值；只规范化 NaN 的表示。"""
    if isinstance(value, float):
        return {'float_hex': value.hex()}
    if isinstance(value, (tuple, list)):
        return [canonical(item) for item in value]
    if isinstance(value, dict):
        return {key: canonical(item) for key, item in value.items()}
    if isinstance(value, SimpleNamespace):
        return canonical(vars(value))
    return value


class TraceRos(FakeRos):
    """把主入口的外部可观察副作用记录下来；不比较私有辅助函数名或栈行号。"""

    def __init__(self, module, scenario):
        """固定单调秒、ROS 纳秒和来源数，分别覆盖正常回调或启动拒绝；不创建 DDS。"""
        super().__init__()
        owner = self
        self.module = module
        self.scenario = scenario
        self.now = 1.0
        self.ros_ns = 10_000_000_025
        self.counts = {self.values['input_topic']: 1, self.values['health_topic']: 1}
        self.callbacks = {}
        self.timer = None
        self.publications = []
        self.details = []
        original_node = self.modules['rclpy.node'].Node
        if scenario == 'missing_identity':
            self.values['source_id'] = ''
        elif scenario == 'missing_topic':
            self.values['input_topic'] = ''
        elif scenario == 'bad_depth':
            self.values['qos_depth'] = 1001
        elif scenario == 'bad_pair_timeout':
            self.values['pair_timeout'] = -1.0

        class Node(original_node):
            """记录构造阶段的外部调用；仍由被测生产代码创建、配对和发布状态。"""

            def __init__(self, name):
                """登记唯一节点名称和所属夹具；无真实资源。"""
                super().__init__(name)
                owner.details.append(['node', name])

            def get_parameter(self, name):
                """记录时钟参数读取；独立场景用 true 检查首个拒绝原因。"""
                owner.details.append(['get_parameter', name])
                return SimpleNamespace(value=owner.scenario == 'sim_time')

            def set_descriptor(self, name, descriptor):
                """记录只读描述符的完整字段，防止可读性抽取放宽配置策略。"""
                owner.details.append(['descriptor', name, vars(descriptor)])

            def declare_parameter(self, name, default, **kwargs):
                """记录参数声明次序、默认值和描述符，返回显式夹具配置。"""
                owner.details.append(['declare', name, default, vars(kwargs['descriptor'])])
                return super().declare_parameter(name, default, **kwargs)

            def create_publisher(self, message_type, topic, qos):
                """只记录 VisionState 发布；消息值由生产适配器实际计算。"""
                owner.details.append(['publisher', message_type.__name__, topic, qos])

                def publish(message):
                    """复制发布时的完整消息，防止后续可变对象复用掩盖差异。"""
                    result = canonical(vars(message))
                    owner.publications.append(result)
                    owner.details.append(['publish', topic, result])

                return SimpleNamespace(publish=publish)

            def create_subscription(self, message_type, topic, callback, qos):
                """记录输入消息身份/QoS，并保留生产回调供确定性事件回放。"""
                owner.details.append(['subscription', message_type.__name__, topic, qos])
                owner.callbacks[topic] = callback
                return object()

            def create_timer(self, period, callback):
                """保存生产超时回调及周期，不用墙钟等待代替测量时间。"""
                owner.details.append(['timer', period])
                owner.timer = callback
                return object()

            def get_logger(self):
                """逐字记录启动和原因变化日志，包括与实际发布的相对次序。"""
                return SimpleNamespace(info=lambda text: owner.details.append(['log', text]))

            def get_clock(self):
                """返回由输入流控制的 ROS 纳秒，保持与单调接收寿命分离。"""
                return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=owner.ros_ns))

            def count_publishers(self, topic):
                """记录逐回调图查询，不允许重构缓存或省略来源数检查。"""
                owner.details.append(['count_publishers', topic, owner.counts[topic]])
                return owner.counts[topic]

            def destroy_node(self):
                """只记录主入口的显式销毁，不替原始基线新增信号行为。"""
                owner.events.append('node.destroy')

        class VisionState:
            """提供 ROS 消息形状，允许被测映射写入完整字段。"""

            def __init__(self):
                """仅给时间戳构造嵌套对象，其他字段必须由生产映射明确写入。"""
                self.stamp = SimpleNamespace(sec=0, nanosec=0)

        self.modules['rclpy.node'].Node = Node
        self.modules['jikl_fc01_system.msg'].VisionState = VisionState
        self.modules['rclpy'].init = self.initialize
        self.modules['rclpy'].spin = self.original_spin
        self.executor.spin_once = self.spin_once
        module.time = SimpleNamespace(monotonic=lambda: self.now)

    def initialize(self, *, args, signal_handler_options=None):
        """两版本都初始化活动 context；信号选项是已单独验证的有意修复，不加入等价轨迹。"""
        self.active = True
        self.events.append('context.init')

    def original_spin(self, node):
        """模拟原 rclpy.spin 的 add→工作→remove 合同，用相同外部输入驱动基线。"""
        try:
            self.executor.add_node(node)
            self.replay()
        finally:
            self.executor.remove_node(node)

    def spin_once(self, *, timeout_sec):
        """当前版本执行同一事件流；最后只请求停止，不能提前关闭活动 context。"""
        self.replay()
        signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)

    def pair(self, nanosec, *, reverse=False, tracking=True):
        """在同一测量纳秒发布一对原始消息，覆盖两种到达顺序及显式跟踪失效。"""
        self.ros_ns = 10_000_000_000 + nanosec
        msg = odometry(position=(1.0, 2.0, 3.0), velocity=(0.2, -0.3, 0.4))
        msg.header.frame_id = self.values['input_frame_id']
        msg.child_frame_id = self.values['input_body_frame_id']
        msg.header.stamp = stamp(10, nanosec)
        meta = health(
            stamp=stamp(10, nanosec),
            sequence=nanosec,
            source_id=self.values['source_id'],
            frame_id=self.values['input_frame_id'],
            body_frame_id=self.values['input_body_frame_id'],
            extrinsics_id=self.values['extrinsics_id'],
            alignment_id=self.values['alignment_id'],
            tracking=tracking,
        )
        calls = [
            (self.callbacks[self.values['health_topic']], meta),
            (self.callbacks[self.values['input_topic']], msg),
        ]
        for callback, payload in reversed(calls) if reverse else calls:
            callback(payload)

    def replay(self):
        """回放健康、重复戳、来源冲突、来源恢复、失效健康和配对超时，顺序不可调换。"""
        self.pair(25)
        self.now += 0.01
        self.pair(25, reverse=True)
        self.counts[self.values['input_topic']] = 2
        self.timer()
        self.counts[self.values['input_topic']] = 1
        self.now += 0.01
        self.pair(50, reverse=True)
        self.now += 0.01
        self.pair(75, tracking=False)
        self.now += 0.01
        self.ros_ns = 10_000_000_100
        pending = odometry()
        pending.header.frame_id = self.values['input_frame_id']
        pending.child_frame_id = self.values['input_body_frame_id']
        pending.header.stamp = stamp(10, 100)
        self.callbacks[self.values['input_topic']](pending)
        self.now += 0.06
        self.ros_ns += 60_000_000
        self.timer()


def trace(path, scenario):
    """加载指定源码并执行一个场景，返回可观察输出和异常；不运行源码文件的 CLI 分支。"""
    spec = importlib.util.spec_from_file_location('fc01_stage2_trace_adapter', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    fixture = TraceRos(module, scenario)
    error = None
    with patch.dict(sys.modules, fixture.modules):
        try:
            module.main([])
        except Exception as exc:
            error = [type(exc).__name__, str(exc)]
    return canonical({'events': fixture.events, 'details': fixture.details, 'error': error})


if __name__ == '__main__':
    print(json.dumps(trace(Path(sys.argv[1]), sys.argv[2]), sort_keys=True))
