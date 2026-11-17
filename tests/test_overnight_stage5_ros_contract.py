# ROS 工程接线的静态合同；不导入 rclpy、不启动 ROS 图，也不运行编译/部署入口。
# 真正生成接口和加载类型支持仍由独立 Humble 构建、安装身份检查与通信 smoke 验证。
from pathlib import Path
import re
import shlex
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'native/src/jikl_fc01_system'


def cmake_commands():
    # 当前受检文件只有单行 # 注释，无 bracket argument/comment；保留引号内字面量。
    # 返回实际调用的参数，而非用注释中可能出现的函数名冒充执行接线。
    text = '\n'.join(
        line
        for line in (PACKAGE / 'CMakeLists.txt').read_text().splitlines()
        if not line.lstrip().startswith('#')
    )
    return [
        (name, shlex.split(arguments))
        for name, arguments in re.findall(r'(\w+)\s*\(([^()]*)\)', text)
    ]


class RosEngineeringContractTests(unittest.TestCase):
    def test_message_generation_links_the_current_package_typesupport(self):
        # 五份公开消息必须来自同一个包；生成字段引用的时间类型也必须声明依赖。
        expected = {
            'VisionState',
            'NavigationCommand',
            'VelocityCommand',
            'VisionHealth',
            'AttitudeThrustPreview',
        }
        commands = cmake_commands()
        generators = [args for name, args in commands if name == 'rosidl_generate_interfaces']
        self.assertEqual(len(generators), 1)
        self.assertEqual(generators[0][0], '${PROJECT_NAME}')
        self.assertEqual(
            {Path(arg).stem for arg in generators[0] if arg.endswith('.msg')}, expected
        )
        self.assertEqual({path.stem for path in (PACKAGE / 'msg').glob('*.msg')}, expected)
        self.assertEqual(generators[0][-2:], ['DEPENDENCIES', 'builtin_interfaces'])
        self.assertIn(
            (
                'rosidl_get_typesupport_target',
                ['fc01_typesupport', '${PROJECT_NAME}', 'rosidl_typesupport_cpp'],
            ),
            commands,
        )
        self.assertIn(
            (
                'target_link_libraries',
                ['fc01_system_node', 'Threads::Threads', '${fc01_typesupport}'],
            ),
            commands,
        )
        self.assertIn(('ament_export_dependencies', ['rosidl_default_runtime']), commands)

    def test_installed_tools_keep_sibling_modules_and_runtime_dependencies(self):
        # 两个可执行入口与两个被导入模块必须安装到同一包目录；相对路径按源码包位置计算。
        wanted = {
            'PROGRAMS': {'vision_odometry_adapter.py', 'velocity_command_diagnostic.py'},
            'FILES': {'vision_state_validation.py', 'vision_resource_budget.py'},
        }
        installs = {}
        for name, args in cmake_commands():
            if name == 'install' and args[0] in wanted:
                kind = args[0]
                boundary = args.index('DESTINATION')
                self.assertEqual(args[boundary + 1 :], ['lib/${PROJECT_NAME}'])
                paths = [
                    Path(arg.replace('${CMAKE_CURRENT_SOURCE_DIR}', str(PACKAGE)))
                    for arg in args[1:boundary]
                ]
                installs.setdefault(kind, set()).update(path.name for path in paths)
                for path in paths:
                    self.assertTrue(path.is_file(), str(path))
                    self.assertEqual(path.resolve().parent, (ROOT / 'native/tools').resolve())
                    if kind == 'PROGRAMS':
                        self.assertTrue(path.read_bytes().startswith(b'#!/usr/bin/env python3\n'))
        self.assertEqual(installs, wanted)

        metadata = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertEqual(metadata.findtext('name'), 'jikl_fc01_system')
        self.assertEqual(metadata.findtext('export/build_type'), 'ament_cmake')
        runtime = {item.text for item in metadata.findall('exec_depend')}
        self.assertTrue(
            {'rclpy', 'rcl_interfaces', 'nav_msgs', 'rosidl_default_runtime'} <= runtime
        )
        self.assertEqual(metadata.findtext('member_of_group'), 'rosidl_interface_packages')
        self.assertIn(
            'rosidl_default_generators',
            {item.text for item in metadata.findall('buildtool_depend')},
        )


if __name__ == '__main__':
    unittest.main()
