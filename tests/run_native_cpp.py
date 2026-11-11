# 主机 C++ 回归入口：编译真实 native/tests/test_*.cpp 并运行每个断言驱动程序。
# 缓存键包含编译器版本、编译参数和全部相关源码字节；不会把旧 ELF 当成当前源码验证结果。
"""Host C++ regression runner with content-addressed compiler artifacts."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

# 构建共享输入摘要，以两个编译任务限制主机资源；全部程序完成后才打印 NATIVE_CPP_PASS。
def run():
    native = ROOT/'native'
    cache = ROOT/'.superpowers/native-host-build'
    cache.mkdir(parents=True, exist_ok=True)
    include = native/'src/jikl_fc01_system/include'
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-pthread', '-I', str(include)]
    # 编译器或严格告警参数变化也必须使缓存失效，而不仅比较源文件时间。
    identity = subprocess.check_output(['g++', '--version'])+repr(flags).encode()
    for p in sorted([*include.rglob('*.hpp'), *native.joinpath('tests').rglob('*.hpp'), *native.joinpath('tests').rglob('*.cpp')]):
        identity += str(p.relative_to(native)).encode()+p.read_bytes()
    common = hashlib.sha256(identity).digest()
    sources = sorted((native/'tests').glob('test_*.cpp'))
    # 每个源文件映射独立内容键，先生成进程专属 pending 文件，再原子发布完整可执行文件。
    def build(source):
        key = hashlib.sha256(common+source.read_bytes()).hexdigest()[:24]
        binary = cache/(source.stem+'_'+key)
        if not binary.exists():
            pending = binary.with_suffix('.pending_'+str(os.getpid()))
            p = subprocess.run(['g++', *flags, str(source), '-o', str(pending)], capture_output=True, text=True)
            if p.returncode:
                raise RuntimeError(source.name+' compile:\n'+p.stdout+p.stderr)
            pending.replace(binary)
        return source, binary
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(build, sources))
    # 编译成功不代表断言通过：逐个实际执行，任一超时或非零退出即中止整轮。
    for source, binary in results:
        p = subprocess.run([str(binary)], capture_output=True, text=True, timeout=20)
        if p.returncode:
            raise RuntimeError(source.name+' run:\n'+p.stdout+p.stderr)
    print('NATIVE_CPP_PASS', len(results), 'executables')

if __name__ == '__main__':
    run()
