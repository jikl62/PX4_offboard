# 历史漏洞探针回放：保持原始C11/C12证据源码，检查旧漏洞断言已经不能成立。
# 这里预期子进程因旧断言失败而SIGABRT；当前安全性结论须结合独立的R4正向断言。
"""Preserve raw vulnerability probes; positive R4 safety tests are separate."""
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


# 关闭core文件并在临时目录编译旧探针；返回码和断言签名必须同时匹配。
def main():
    # 旧探针预期中止，禁止产生core转储，避免污染工作目录。
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    with tempfile.TemporaryDirectory(prefix="fc01-c11-c12-") as directory:
        binary = Path(directory) / "original_cases"
        command = [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(ROOT / "src/jikl_fc01_system/include"),
            "-I",
            str(ROOT / "tests"),
            str(ROOT / "tests/c_independent_repro/C_R3_CASE_ENTRY.cpp"),
            "-o",
            str(binary),
        ]
        subprocess.run(command, check=True)
        env = dict(os.environ, GLIBC_TUNABLES="glibc.malloc.check=0")
        for case, signature in [
            ("c11", "p.accepted&&p.command400_kind"),
            ("c12", "r.pending(400).rejected&&phase"),
        ]:
            result = subprocess.run(
                ["stdbuf", "-oL", str(binary), case], capture_output=True, text=True, env=env
            )
            print("ORIGINAL_UNCHANGED_" + case.upper(), flush=True)
            print(result.stdout + result.stderr, end="", flush=True)
            print(
                "ORIGINAL_VULNERABILITY_ASSERTION_EXIT_CODE=" + str(result.returncode), flush=True
            )
            # 仅接受SIGABRT及本案例的断言文字；其他崩溃或失败不能伪装成修复证据。
            if result.returncode != -6 or signature not in result.stderr:
                return 1
            print("EXPECTED_OLD_VULNERABILITY_ASSERTION_NO_LONGER_HOLDS=1", flush=True)
    print(
        "Original aborts are diagnostic evidence; R4 positive safety assertions are the success oracle.",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
