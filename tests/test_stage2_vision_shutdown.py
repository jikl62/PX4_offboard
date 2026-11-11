"""不连接 DDS 的适配器退出回归：信号只请求停止，正常运行错误仍向上传播。"""

import importlib.util
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_adapter():
    """加载当前适配器而不导入 ROS；返回模块，唯一副作用是登记测试模块名。"""
    spec = importlib.util.spec_from_file_location(
        'fc01_stage2_shutdown_adapter', ROOT / 'native/tools/vision_odometry_adapter.py'
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeRos:
    """模拟主线程 ROS 生命周期，保存有序资源记录；不创建真实通信端点。"""

    def __init__(self, *, spin_error=None, destroy_error=None, shutdown_error=None):
        """指定可选故障注入点；时间和坐标转换不参与此退出合同测试。"""
        self.events = []
        self.active = False
        self.node = None
        self.spin_error = spin_error
        self.destroy_error = destroy_error
        self.shutdown_error = shutdown_error
        self.previous_handlers = {
            item: signal.getsignal(item) for item in (signal.SIGINT, signal.SIGTERM)
        }
        self.values = {
            'source_id': 'shutdown-fixture',
            'input_frame_id': 'fixture_ned',
            'input_body_frame_id': 'fixture_frd',
            'world_convention': 'NED',
            'body_convention': 'FRD',
            'extrinsics_id': 'identity',
            'alignment_id': 'identity',
            'body_from_sensor_wxyz': [1.0, 0.0, 0.0, 0.0],
            'world_alignment_wxyz': [1.0, 0.0, 0.0, 0.0],
            'sensor_position_body_frd': [0.0, 0.0, 0.0],
            'world_translation_ned': [0.0, 0.0, 0.0],
            'input_topic': '/fc01_demo/shutdown/odom',
            'health_topic': '/fc01_demo/shutdown/health',
            'output_topic': '/fc01_demo/shutdown/state',
        }
        owner = self

        class Node:
            """只记录构造和资源释放；参数缺省仍由生产入口声明。"""

            def __init__(self, name):
                """登记节点所有者；此假节点不拥有线程或套接字。"""
                owner.events.append('node.create')
                owner.node = self

            def get_parameter(self, name):
                """返回生产入口预声明的 use_sim_time，保持系统时间域为 false。"""
                return SimpleNamespace(value=False)

            def set_descriptor(self, *args):
                """接受只读描述符；本组测试不修改任何参数。"""

            def declare_parameter(self, name, default, **kwargs):
                """返回显式夹具配置或生产默认值，不推断传感器身份。"""
                return SimpleNamespace(value=owner.values.get(name, default))

            def create_publisher(self, *args):
                """返回空发布器占位，禁止产生任何 ROS 输出。"""
                return object()

            def create_subscription(self, *args):
                """保存订阅对象占位；不触发真实回调。"""
                return object()

            def create_timer(self, *args):
                """返回定时器占位，不启动真实时间源。"""
                return object()

            def get_logger(self):
                """日志仍由生产入口调用；此处只收集其调用顺序。"""
                return SimpleNamespace(info=lambda text: owner.events.append('log'))

            def destroy_node(self):
                """记录销毁并注入可选故障；检查重复信号不打断清理。"""
                owner.events.append('node.destroy')
                for signum in (signal.SIGINT, signal.SIGTERM):
                    signal.getsignal(signum)(signum, None)
                if owner.destroy_error is not None:
                    raise owner.destroy_error

        class Executor:
            """只执行一次受控工作，模拟信号在一项回调内部到达。"""

            context = SimpleNamespace(ok=lambda: owner.active)

            def add_node(self, node):
                """记录节点加入；资源释放必须先移出再销毁。"""
                owner.events.append('executor.add')

            def spin_once(self, *, timeout_sec):
                """模拟回调：信号只能置位，必须在本方法返回后清理资源。"""
                owner.events.append('callback.begin')
                if owner.spin_error is not None:
                    if not owner.active:
                        raise AssertionError('error must be injected in an active context')
                    raise owner.spin_error
                for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGINT):
                    signal.getsignal(signum)(signum, None)
                if not owner.active:
                    raise AssertionError('signal handler closed a running callback context')
                owner.events.append('callback.end')

            def remove_node(self, node):
                """记录移出节点，不改变 context 状态。"""
                owner.events.append('executor.remove')

        self.executor = Executor()
        self.modules = {}
        for name in (
            'rclpy',
            'rclpy.node',
            'rclpy.executors',
            'rclpy.signals',
            'rclpy.qos',
            'rcl_interfaces',
            'rcl_interfaces.msg',
            'nav_msgs',
            'nav_msgs.msg',
            'jikl_fc01_system',
            'jikl_fc01_system.msg',
        ):
            self.modules[name] = ModuleType(name)
        ros = self.modules['rclpy']
        ros.init = self.init
        ros.ok = lambda: self.active
        ros.shutdown = self.shutdown
        ros.get_global_executor = lambda: self.executor
        self.modules['rclpy.node'].Node = Node
        self.modules['rclpy.executors'].ExternalShutdownException = type(
            'ExternalShutdownException', (Exception,), {}
        )
        self.modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO=0)
        qos = self.modules['rclpy.qos']
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
        qos.DurabilityPolicy = SimpleNamespace(VOLATILE=1)
        self.modules['rcl_interfaces.msg'].ParameterDescriptor = SimpleNamespace
        self.modules['rcl_interfaces.msg'].ParameterType = SimpleNamespace(PARAMETER_BOOL=1)
        self.modules['nav_msgs.msg'].Odometry = type('Odometry', (), {})
        for name in ('VisionHealth', 'VisionState'):
            setattr(self.modules['jikl_fc01_system.msg'], name, type(name, (), {}))

    def init(self, *, args, signal_handler_options):
        """仅接受禁用 ROS 异步信号处理的初始化，记录 context 进入活动状态。"""
        if signal_handler_options != self.modules['rclpy.signals'].SignalHandlerOptions.NO:
            raise AssertionError('ROS asynchronous context shutdown must be disabled')
        self.events.append('context.init')
        self.active = True

    def shutdown(self):
        """记录关闭并注入可选故障；主入口仍必须恢复调用者的信号处理器。"""
        self.events.append('context.shutdown')
        self.active = False
        if self.shutdown_error is not None:
            raise self.shutdown_error

    def assert_handlers_restored(self, testcase):
        """对比入口调用前的两个处理器对象，不用字符串描述替代身份检查。"""
        for signum, previous in self.previous_handlers.items():
            testcase.assertIs(signal.getsignal(signum), previous)


class VisionShutdownTests(unittest.TestCase):
    """验证停止信号、活动异常和资源异常的独立路径，不宽松吞掉 RuntimeError。"""

    def test_signals_finish_callback_then_release_resources_in_order(self):
        """多次停止请求仍先完成回调，再依序移出、销毁节点、关闭上下文。"""
        fixture = FakeRos()
        with patch.dict(sys.modules, fixture.modules):
            load_adapter().main([])
        self.assertEqual(
            [item for item in fixture.events if item != 'log'],
            [
                'context.init',
                'node.create',
                'executor.add',
                'callback.begin',
                'callback.end',
                'executor.remove',
                'node.destroy',
                'context.shutdown',
            ],
        )
        self.assertFalse(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_active_runtime_error_is_not_swallowed(self):
        """活跃回调中的真正 RuntimeError 原对象向上传播，同时按顺序释放资源。"""
        error = RuntimeError('stage2 active callback marker')
        fixture = FakeRos(spin_error=error)
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(RuntimeError) as caught:
            load_adapter().main([])
        self.assertIs(caught.exception, error)
        self.assertEqual(
            fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
        )
        self.assertFalse(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_destroy_failure_still_closes_context_and_restores_handlers(self):
        """节点销毁失败不阻止后续 context 清理；错误仍暴露给调用方。"""
        error = RuntimeError('stage2 destroy marker')
        fixture = FakeRos(destroy_error=error)
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(RuntimeError) as caught:
            load_adapter().main([])
        self.assertIs(caught.exception, error)
        self.assertEqual(fixture.events[-2:], ['node.destroy', 'context.shutdown'])
        self.assertFalse(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_shutdown_failure_still_restores_handlers(self):
        """context 关闭故障保持可见，两个入口前信号处理器仍完整恢复。"""
        error = RuntimeError('stage2 shutdown marker')
        fixture = FakeRos(shutdown_error=error)
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(RuntimeError) as caught:
            load_adapter().main([])
        self.assertIs(caught.exception, error)
        fixture.assert_handlers_restored(self)

    def test_invalid_startup_configuration_is_not_swallowed(self):
        """未完成配置仍拒绝启动，并关闭已经创建的 ROS 上下文。"""
        fixture = FakeRos()
        fixture.values['source_id'] = ''
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(ValueError):
            load_adapter().main([])
        self.assertFalse(fixture.active)
        self.assertEqual(fixture.events[-1], 'context.shutdown')
        fixture.assert_handlers_restored(self)


if __name__ == '__main__':
    unittest.main()
