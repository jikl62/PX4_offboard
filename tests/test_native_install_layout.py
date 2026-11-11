"""离线检查独立 native 安装器复制后的真实 CMake 工具安装路径。"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT=Path(__file__).resolve().parents[1]


class NativeInstallLayoutTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cmake'),'requires installed CMake')
    def test_copied_workspace_installs_all_declared_python_tools(self):
        installer=(ROOT/'native/scripts/install.sh').read_text()
        # 只执行源码中的工作区复制段，不运行平台探针、ROS、编译或服务入口。
        copy_block=installer.split('mkdir -p "$release/workspace/src"',1)[1].split('set +u',1)[0]
        copy_block='mkdir -p "$release/workspace/src"'+copy_block
        cmake=(ROOT/'native/src/jikl_fc01_system/CMakeLists.txt').read_text()
        # 使用生产 CMake 的实际工具安装声明，省去与本测试无关的 ROS 依赖发现。
        install_blocks=re.findall(r'install\((?:PROGRAMS|FILES).*?\)',cmake,re.S)
        self.assertTrue(install_blocks)
        with tempfile.TemporaryDirectory(prefix='fc01-native-layout-') as d:
            folder=Path(d);release=folder/'release';messages=folder/'px4_msgs'
            messages.mkdir();(messages/'package.xml').write_text('<package/>')
            package=release/'src/jikl_fc01_system';package.mkdir(parents=True)
            (package/'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.16)\nproject(jikl_fc01_system NONE)\n'+
                '\n'.join(install_blocks)+'\n')
            shutil.copytree(ROOT/'native/tools',release/'tools',
                            ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            # 路径通过位置参数传递，含空格的目录也不会变成额外 shell 命令。
            copied=subprocess.run(['bash','-eu','-c','release=$1\nmessages=$2\n'+copy_block,
                                   'fc01-layout',str(release),str(messages)],capture_output=True,text=True)
            self.assertEqual(copied.returncode,0,copied.stderr)
            build=folder/'build';prefix=folder/'install'
            configured=subprocess.run(['cmake','-S',str(release/'workspace/src/jikl_fc01_system'),
                                       '-B',str(build),'-DCMAKE_INSTALL_PREFIX='+str(prefix)],
                                      capture_output=True,text=True)
            self.assertEqual(configured.returncode,0,configured.stderr)
            installed=subprocess.run(['cmake','--install',str(build)],capture_output=True,text=True)
            self.assertEqual(installed.returncode,0,installed.stderr)
            for name in ('vision_odometry_adapter.py','velocity_command_diagnostic.py',
                         'vision_state_validation.py'):
                with self.subTest(tool=name):
                    self.assertEqual((prefix/'lib/jikl_fc01_system'/name).read_bytes(),
                                     (ROOT/'native/tools'/name).read_bytes())


if __name__=='__main__':
    unittest.main()
