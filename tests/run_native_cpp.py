"""Host C++ regression runner with content-addressed compiler artifacts."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def run():
    native = ROOT/'native'
    cache = ROOT/'.superpowers/native-host-build'
    cache.mkdir(parents=True, exist_ok=True)
    include = native/'src/jikl_fc01_system/include'
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-pthread', '-I', str(include)]
    identity = subprocess.check_output(['g++', '--version'])+repr(flags).encode()
    for p in sorted([*include.rglob('*.hpp'), *native.joinpath('tests').rglob('*.hpp'), *native.joinpath('tests').rglob('*.cpp')]):
        identity += str(p.relative_to(native)).encode()+p.read_bytes()
    common = hashlib.sha256(identity).digest()
    sources = sorted((native/'tests').glob('test_*.cpp'))
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
    for source, binary in results:
        p = subprocess.run([str(binary)], capture_output=True, text=True, timeout=20)
        if p.returncode:
            raise RuntimeError(source.name+' run:\n'+p.stdout+p.stderr)
    print('NATIVE_CPP_PASS', len(results), 'executables')

if __name__ == '__main__':
    run()
