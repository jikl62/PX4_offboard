#!/usr/bin/env python3
# UART 被动证据采集器：只读文件元数据和白名单查询命令，生成供人工复核的报告。
# 不打开串口、不改 overlay/参数/服务；用户历史陈述、固件源码映射与本机当下观测分别记录。
"""Passive UART evidence collection. Never opens a serial port or changes a service.

Run with ordinary Python 3: python3 uart_preflight.py --output NEW_DIRECTORY.
Only the two read-only command forms in COMMANDS below may run. The filesystem
and command runner can be injected by tests; this is not a UART loopback test.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess


# 单次文件和命令回读限制 64 KiB；命令最长等待 4 秒，目录与符号链接遍历另有容量/深度限制。
READ_LIMIT = 65536
COMMAND_TIMEOUT = 4
COMMANDS = ("systemctl show", "fuser --")
# 这是用户报告的物理接线假设，不是采集器确认的现场事实；工作主链路策略保持 DO_NOT_CHANGE。
DECLARED_USER_MAPPING = {
    "evidence_class": "USER_REPORTED_PHYSICAL_CONNECTION",
    "reported_date": "2026-11-06",
    "fc01_wiring_verification": "USER_REPORT_REQUIRES_EVIDENCE_RECONCILIATION",
    "primary": {
        "board": "OrangePi Zero3W V1.2",
        "uart": "UART2",
        "device": "/dev/ttyS2",
        "tx_pin": 11,
        "rx_pin": 13,
        "baud": 115200,
        "peer": "FC01 UART6 (USART6), RX6/TX6",
        "peer_hardware_uart_claim": "USART6",
        "peer_physical_connector_claim": None,
        "peer_verification": "USER_REPORTED_PHYSICAL_CONNECTION",
        "change_policy": "DO_NOT_CHANGE",
    },
    "candidate": {
        "board": "OrangePi Zero3W V1.2",
        "uart": "UART6",
        "device": "/dev/ttyS6",
        "tx_pin": 24,
        "rx_pin": 23,
        "ground_pin": 25,
        "peer": "FC01 UART4, physical TELEM3, RX4/TX4",
        "peer_hardware_uart_claim": "UART4",
        "peer_physical_connector_claim": "TELEM3",
        "peer_verification": "PENDING_ON_DEVICE",
        "activation": "NOT_ENABLED_BY_THIS_TOOL",
    },
    "notice": "Latest user-reported physical connection. Supplied NSH confirms the DDS startup device "
    "/dev/ttyS3, not its physical connector; reconcile the separate runtime/source evidence "
    "before enabling the candidate. A physical connector name is not a PX4 logical port selector.",
}

# Dated source reference, never an observation of the attached flight controller.
# The script remains standalone: this table is explanatory, not runtime config.
# 这是指定提交的固件源码参考。STM32 UART 名、PX4 逻辑端口和 Linux tty 编号不能互相直接推导。
FC01_SOURCE_REFERENCE = {
    "evidence_class": "LOCAL_FIRMWARE_SOURCE_REFERENCE",
    "reviewed_date": "2026-11-06",
    "source_git_head": "07983bfb894b9372d7520d43ac10dc792d0b4b9f",
    "source_has_custom_changes": True,
    "on_device_firmware_match": "NOT_VERIFIED",
    "ports": [
        {
            "stm32_hardware_uart": "UART4",
            "px4_logical_port": "TELEM3",
            "px4_selector": 103,
            "nuttx_device": "/dev/ttyS3",
            "baud_parameter": "SER_TEL3_BAUD",
        },
        {
            "stm32_hardware_uart": "USART6",
            "px4_logical_port": "URT6",
            "px4_selector": 6,
            "nuttx_device": "/dev/ttyS5",
            "baud_parameter": "SER_URT6_BAUD",
        },
        {
            "stm32_hardware_uart": "UART7",
            "px4_logical_port": "TELEM4",
            "px4_selector": 104,
            "nuttx_device": "/dev/ttyS6",
            "baud_parameter": "SER_TEL4_BAUD",
        },
    ],
    "notice": "In the reviewed source, USART6 uses URT6/6; PX4 logical TELEM4/104 uses UART7. "
    "Do not derive the PX4 selector from a physical TELEM4 label or OrangePi ttyS6. "
    "This tool neither reads nor validates live PX4 parameters.",
}

# 历史终端结果仅作为用户陈述保留，不参与替代当前 Linux 检查，也不授权启用候选串口。
USER_REPORTED_PROGRESS = {
    "evidence_class": "USER_REPORTED_PROGRESS",
    "reported_date": "2026-11-06",
    "orangepi": {
        "primary_device_registered": "/dev/ttyS2",
        "uart6_overlay_enabled": True,
        "removed_conflicting_overlay": "spi3-cs0-cs1-spidev",
        "reboot_completed": True,
        "candidate_device_registered": "/dev/ttyS6",
        "candidate_driver": "uart-ng",
        "navigation_service_release": "20261103",
        "navigation_service_state": "active",
        "candidate_physical_loopback_test": "NOT_COMPLETED",
        "candidate_fc01_connection_test": "NOT_COMPLETED",
    },
    "notice": "User reports these results from the OrangePi terminal; the collector has not "
    "independently inspected that terminal output. These claims never replace "
    "ACTUAL_LOCAL_OBSERVATION or affect its checks and summary.",
}

# 历史 NSH 参数与启动日志说明的是当时运行端点；不能证明此刻的接线、占用或候选链路。
USER_PROVIDED_FC01_RUNTIME = {
    "evidence_class": "USER_PROVIDED_NSH_OUTPUT",
    "provided_date": "2026-11-06",
    "capture_time": "UNKNOWN",
    "board": "DAMIAO_DM_FC01",
    "firmware_git_hash": "07983bfb894b9372d7520d43ac10dc792d0b4b9f",
    "firmware_build_date": "Oct 27 2026 00:49:27",
    "UXRCE_DDS_CFG": 103,
    "client_status": "Running, connected, serial",
    "transport_inits": 1,
    "startup_device": "/dev/ttyS3",
    "startup_baud": 115200,
    "normalized_startup_log_signals": [
        "Starting UXRCE-DDS Client on /dev/ttyS3",
        "init serial /dev/ttyS3 @ 115200 baud",
    ],
    "notice": "Dated evidence supplied by the user, not live discovery by this collector. "
    "Startup device and parameters support the primary DDS endpoint at that time; "
    "they do not validate the candidate link, physical pin continuity, or current occupancy.",
}


# 只比对三类有日期的证据是否矛盾；即便一致，也只得到历史一致，不形成硬件就绪结论。
def reconcile_fc01_evidence(declaration, runtime, source):
    """Compare dated evidence only; never discover a live port or grant readiness."""
    primary = declaration.get("primary", {}).get("peer_hardware_uart_claim")
    candidate = declaration.get("candidate", {}).get("peer_hardware_uart_claim")
    result = {
        "status": "UNKNOWN",
        "evidence_class": "DATED_EVIDENCE_COMPARISON",
        "reported_primary_hardware_uart": primary,
        "reported_candidate_hardware_uart": candidate,
        "source_mapped_runtime_hardware_uart": None,
        "runtime_selector": runtime.get("UXRCE_DDS_CFG"),
        "runtime_device": runtime.get("startup_device"),
        "candidate_availability": "UNRESOLVED_DO_NOT_ENABLE",
        "notice": "A local source mapping does not prove the installed binary or physical wiring. "
        "Even consistent dated evidence does not validate a candidate link.",
    }
    ports = [
        row
        for row in source.get("ports", [])
        if row.get("px4_selector") == result["runtime_selector"]
    ]
    if len(ports) != 1 or not primary or not result["runtime_device"]:
        result["reason"] = "Missing or ambiguous source/runtime/physical-connection evidence."
        return result
    port = ports[0]
    if port.get("nuttx_device") != result["runtime_device"]:
        result.update(
            status="CONFLICT",
            reason="Runtime selector and startup device disagree with the reviewed source.",
        )
        return result
    mapped = port.get("stm32_hardware_uart")
    result["source_mapped_runtime_hardware_uart"] = mapped
    result["candidate_matches_source_mapped_runtime"] = candidate == mapped
    if not mapped:
        result["reason"] = "Source hardware UART mapping is unavailable."
    elif primary != mapped:
        result.update(
            status="CONFLICT",
            reason="User-reported physical primary UART differs from source-mapped NSH runtime.",
        )
    else:
        result.update(
            status="CONSISTENT_DATED_EVIDENCE_ONLY",
            reason="No mismatch in these records; live wiring remains unverified.",
        )
    return result


# 将 tty 简名规范化为 /dev 路径；拒绝路径上跳和 shell 字符，后续命令始终以参数数组调用。
def device_name(value):
    """Accept a tty basename or a confined /dev path, never shell syntax."""
    value = str(value)
    if not value.startswith("/"):
        value = "/dev/" + value
    parts = PurePosixPath(value).parts
    if (
        not value.startswith("/dev/")
        or ".." in parts
        or "." in value.split("/")
        or not re.fullmatch(r"/dev/[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", value)
    ):
        raise ValueError("device must be a basename or a confined /dev path")
    return value


# 可注入文件系统根供离线夹具测试；绝对内核符号链接也按该虚拟根解释，避免读到宿主机路径。
class SystemPaths:
    """Resolve kernel symlinks inside a virtual root, with bounded traversal.

    Absolute symlinks are interpreted inside rootfs, including in fixtures. File
    contents can only be read under the listed metadata directories, never /dev.
    proc_root/sys_root/dev_root are function-only injection points, not CLI flags.
    """

    def __init__(self, rootfs="/", *, proc_root=None, sys_root=None, dev_root=None):
        self.rootfs = Path(rootfs).resolve()
        self.mounts = {
            "/proc": Path(proc_root).resolve() if proc_root else self.rootfs / "proc",
            "/sys": Path(sys_root).resolve() if sys_root else self.rootfs / "sys",
            "/dev": Path(dev_root).resolve() if dev_root else self.rootfs / "dev",
        }
        self.is_host = self.rootfs == Path("/") and all(
            path == Path(prefix) for prefix, path in self.mounts.items()
        )

    # 把逻辑 /proc、/sys、/dev 映射到对应注入根；其他元数据路径相对 rootfs 解析。
    def physical(self, logical):
        for prefix, root in self.mounts.items():
            if logical == prefix or logical.startswith(prefix + "/"):
                return root / logical[len(prefix) :].lstrip("/")
        return self.rootfs / logical.lstrip("/")

    # 逐组件解析符号链接，最多 32 次重启；最终路径须留在允许范围，既保留链路证据又限制跳转。
    def resolve(self, logical, *, scopes=None):
        if not logical.startswith("/") or ".." in PurePosixPath(logical).parts:
            raise ValueError("absolute normalized metadata path required")
        current = logical
        links = []
        for _ in range(32):
            parts = PurePosixPath(current).parts[1:]
            restart = False
            for index in range(len(parts)):
                prefix = "/" + "/".join(parts[: index + 1])
                node = self.physical(prefix)
                info = node.lstat()
                if stat.S_ISLNK(info.st_mode):
                    target = os.readlink(node)
                    target = (
                        target
                        if target.startswith("/")
                        else str(PurePosixPath(prefix).parent / target)
                    )
                    current = os.path.normpath(target + "/" + "/".join(parts[index + 1 :]))
                    links.append({"path": prefix, "target": target})
                    restart = True
                    break
            if restart:
                continue
            if scopes and not any(current == p or current.startswith(p + "/") for p in scopes):
                raise ValueError("symlink leaves allowed metadata scope")
            return self.physical(current), current, links
        raise ValueError("symlink traversal limit exceeded")

    # 只允许元数据目录里的普通文件；NOFOLLOW 与打开后 fstat 再次核对类型，绝不读取 /dev 串口内容。
    def read(self, logical, limit=READ_LIMIT):
        # A malformed sysfs/boot symlink must not redirect metadata collection
        # into unrelated directories (especially /dev or sensitive /etc files).
        scopes = (
            ("/proc/device-tree", "/sys/firmware/devicetree/base")
            if logical.startswith("/proc/device-tree/")
            else ("/" + logical.lstrip("/").split("/", 1)[0],)
        )
        if scopes[0] not in ("/etc", "/boot", "/proc", "/sys", "/proc/device-tree"):
            return {"status": "unknown", "path": logical, "reason": "metadata scope is not allowed"}
        try:
            path, resolved, _ = self.resolve(logical, scopes=scopes)
            if not stat.S_ISREG(path.lstat().st_mode):
                raise ValueError("metadata source is not a regular file")
            fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK)
            try:
                if not stat.S_ISREG(os.fstat(fd).st_mode):
                    raise ValueError("metadata source changed type")
                chunks, remaining = [], limit + 1
                while remaining:
                    data = os.read(fd, min(remaining, 8192))
                    if not data:
                        break
                    chunks.append(data)
                    remaining -= len(data)
                data = b"".join(chunks)
            finally:
                os.close(fd)
            if len(data) > limit:
                return {"status": "unknown", "path": logical, "reason": "read limit exceeded"}
            return {"status": "observed", "path": logical, "resolved": resolved, "data": data}
        except (OSError, ValueError) as error:
            return {
                "status": "unknown",
                "path": logical,
                "reason": type(error).__name__ + ": " + str(error),
            }

    # 目录枚举有数量上限，失败返回未知；不把权限不足、过多条目等失败解释为空目录。
    def names(self, logical, limit=256):
        try:
            path, _, _ = self.resolve(logical, scopes=("/sys", "/proc", "/boot"))
            with os.scandir(path) as entries:
                names = []
                for entry in entries:
                    names.append(entry.name)
                    if len(names) > limit:
                        raise ValueError("directory entry limit exceeded")
            return sorted(names)
        except (OSError, ValueError):
            return None


# 将已读取的字节解码成文本，保留状态和来源路径；解码替换不等于验证内容的硬件含义。
def text_evidence(row):
    row = dict(row)
    if "data" in row:
        row["value"] = row.pop("data").decode("utf-8", errors="replace").strip("\x00\n")
    return row


# 设备树字符串列表由 NUL 分隔，不能当作普通换行文本；未知观测继续保持未知。
def strings_evidence(row):
    row = dict(row)
    if "data" in row:
        row["value"] = [
            x.decode("utf-8", errors="replace")
            for x in row.pop("data").rstrip(b"\x00").split(b"\x00")
            if x
        ]
    return row


# 这里只 stat 字符设备类型及主/次设备号、inode，记录别名；从未 open 串口。
def inspect_device(paths, name):
    try:
        path, resolved, links = paths.resolve(name, scopes=("/dev",))
        info = path.stat()
        char = stat.S_ISCHR(info.st_mode)
        return {
            "status": "observed" if char else "not_ready",
            "path": name,
            "resolved": resolved,
            "symlinks": links,
            "exists": True,
            "type": "character_device" if char else "non_character_device",
            "major": os.major(info.st_rdev) if char else None,
            "minor": os.minor(info.st_rdev) if char else None,
            "identity": [info.st_dev, info.st_ino],
            "notice": "Only stat metadata was read; the device was never opened.",
        }
    except (OSError, ValueError) as error:
        return {
            "status": "unknown",
            "path": name,
            "exists": None,
            "reason": type(error).__name__ + ": " + str(error),
        }


# 同 inode 或相同字符设备主/次号都表示同一设备；候选与主链路相同即拒绝独立性判断。
def same_device(candidate, primary):
    if "identity" not in candidate or "identity" not in primary:
        return {"status": "unknown", "same_device": None}
    equal = candidate["identity"] == primary["identity"] or (
        candidate["type"] == primary["type"] == "character_device"
        and (candidate["major"], candidate["minor"]) == (primary["major"], primary["minor"])
    )
    return {"status": "not_ready" if equal else "observed", "same_device": equal}


# 沿 tty 设备父链最多向上十层，查实际驱动与设备树绑定；不能用系统“已注册驱动”代替实际绑定。
def inspect_binding(paths, name):
    tty = PurePosixPath(name).name
    base = "/sys/class/tty/" + tty
    result = {
        "status": "unknown",
        "tty": tty,
        "driver": {"status": "unknown"},
        "of_node": {"status": "unknown"},
    }
    try:
        _, device, _ = paths.resolve(base + "/device", scopes=("/sys",))
        result["device_path"] = device
        current = device
        # A tty's platform driver and of_node can be bound on an ancestor.
        for _ in range(10):
            if current in ("/", "/sys", "/sys/devices"):
                break
            for key in ("driver", "of_node"):
                if result[key]["status"] == "observed":
                    continue
                try:
                    source = current + "/" + key
                    if not stat.S_ISLNK(paths.physical(source).lstat().st_mode):
                        continue
                    target, resolved, _ = paths.resolve(
                        source, scopes=("/sys", "/proc/device-tree")
                    )
                    if not target.is_dir():
                        continue
                    result[key] = {
                        "status": "observed",
                        "path": resolved,
                        "binding_path": current + "/" + key,
                    }
                except (OSError, ValueError):
                    pass
            current = str(PurePosixPath(current).parent)
        if all(result[key]["status"] == "observed" for key in ("driver", "of_node")):
            result["status"] = "observed"
    except (OSError, ValueError) as error:
        result["reason"] = type(error).__name__ + ": " + str(error)
    return result


# 优先选择内核实时设备树根；两个候选路径均不可读时保持未知，不猜测板卡。
def find_dt_root(paths):
    for root in ("/sys/firmware/devicetree/base", "/proc/device-tree"):
        try:
            path, resolved, _ = paths.resolve(root, scopes=("/sys", "/proc/device-tree"))
            if path.is_dir():
                return resolved
        except (OSError, ValueError):
            pass
    return None


# pinctrl-0 是大端 32 位 phandle 数组；有界扫描设备树寻找对应组并读取功能和引脚属性。
def pinctrl_evidence(paths, root, node):
    handles_row = paths.read(node + "/pinctrl-0", 256)
    result = {
        "status": "unknown",
        "names": strings_evidence(paths.read(node + "/pinctrl-names", 1024)),
        "groups": [],
        "scan_complete": False,
    }
    if handles_row["status"] != "observed":
        result["reason"] = handles_row.get("reason", "pinctrl-0 unavailable")
        return result
    raw = handles_row["data"]
    if not raw or len(raw) % 4:
        result["reason"] = "pinctrl-0 is not a nonempty phandle array"
        return result
    handles = [int.from_bytes(raw[i : i + 4], "big") for i in range(0, len(raw), 4)]
    result["phandles"] = handles
    found, queue, scanned = {}, [(root, 0)], 0
    # 扫描最多 2048 个节点、深度 32；scan_complete 明示是否扫完，找不到的句柄仍记未知。
    while queue and scanned < 2048:
        current, depth = queue.pop()
        scanned += 1
        for property_name in ("phandle", "linux,phandle"):
            row = paths.read(current + "/" + property_name, 4)
            if row["status"] == "observed" and len(row["data"]) == 4:
                value = int.from_bytes(row["data"], "big")
                if value in handles:
                    found[value] = current
        if depth < 32:
            names = paths.names(current, 256)
            if names is None:
                continue
            for name in names:
                child = current + "/" + name
                # DT hierarchy should contain directories and properties, not links.
                try:
                    info = paths.physical(child).lstat()
                    if stat.S_ISDIR(info.st_mode):
                        queue.append((child, depth + 1))
                except OSError:
                    pass
    result["scan_complete"] = not queue
    for handle in handles:
        if handle not in found:
            result["groups"].append({"status": "unknown", "phandle": handle})
            continue
        group = {"phandle": handle, "path": found[handle]}
        for key in ("function", "pins", "groups", "allwinner,function", "allwinner,pins"):
            group[key] = strings_evidence(paths.read(found[handle] + "/" + key, 8192))
        group["status"] = (
            "observed"
            if any(group[k].get("value") for k in ("function", "allwinner,function"))
            and any(group[k].get("value") for k in ("pins", "groups", "allwinner,pins"))
            else "unknown"
        )
        result["groups"].append(group)
    if result["groups"] and all(row["status"] == "observed" for row in result["groups"]):
        result["status"] = "observed"
    return result


# 同时保留 serial 别名和候选 tty 的真实 of_node 绑定；别名数字本身不能证明物理排针。
def inspect_dt(paths, root, binding):
    result = {
        "status": "unknown",
        "root": root,
        "aliases": [],
        "notice": "A serial alias number alone does not prove a physical UART or header pin mapping.",
    }
    if root is None:
        result["reason"] = "Live device tree unavailable"
        return result
    names = paths.names(root + "/aliases", 128)
    for name in names or []:
        if re.fullmatch(r"serial[0-9]+", name):
            row = text_evidence(paths.read(root + "/aliases/" + name, 2048))
            row["alias"] = name
            target = row.get("value", "")
            if target.startswith("/") and ".." not in PurePosixPath(target).parts:
                node = root + target
                row["node_status"] = text_evidence(paths.read(node + "/status", 1024))
                row["compatible"] = strings_evidence(paths.read(node + "/compatible", 4096))
                try:
                    _, row["resolved_node"], _ = paths.resolve(
                        node, scopes=("/sys", "/proc/device-tree")
                    )
                except (OSError, ValueError):
                    row["resolved_node"] = None
            result["aliases"].append(row)
    node = binding.get("of_node", {}).get("path")
    if not node or not (node == root or node.startswith(root + "/")):
        result["reason"] = "Candidate tty has no observed binding into the live device tree"
        return result
    result["bound_node"] = node
    result["compatible"] = strings_evidence(paths.read(node + "/compatible", 4096))
    result["status_chain"] = []
    current = node
    while current != root:
        row = text_evidence(paths.read(current + "/status", 1024))
        result["status_chain"].append(row)
        current = str(PurePosixPath(current).parent)
    # 节点或祖先显式 disabled 即不就绪；祖先缺 status 按设备树约定处理，但本节点缺失仍保守记未知。
    disabled = any(
        row.get("value", "").lower() not in ("", "okay", "ok")
        for row in result["status_chain"]
        if row["status"] == "observed"
    )
    # Missing ancestor status means enabled by DT convention. The bound UART's
    # own missing/permission-denied status stays unknown in this conservative audit.
    own_status = result["status_chain"][0] if result["status_chain"] else {"status": "unknown"}
    uncertain_ancestor = any(
        row["status"] == "unknown" and not row.get("reason", "").startswith("FileNotFoundError:")
        for row in result["status_chain"][1:]
    )
    result["status"] = (
        "not_ready"
        if disabled
        else (
            "observed"
            if own_status["status"] == "observed"
            and own_status.get("value") in ("okay", "ok")
            and bool(result["compatible"].get("value"))
            and not uncertain_ancestor
            else "unknown"
        )
    )
    result["pinctrl"] = pinctrl_evidence(paths, root, node)
    return result


# 仅放行完全匹配的 systemctl show 与 fuser -- 参数形状，无 shell、sudo 或标准输入；未知命令拒绝。
def run_readonly(argv):
    """No shell, sudo, stdin, service mutation, or unbounded subprocess output."""
    allowed = (
        len(argv) == 8
        and argv[0:2] == ["systemctl", "show"]
        and re.fullmatch(r"serial-getty@[A-Za-z0-9_.-]+\.service", argv[2])
        and argv[3:]
        == [
            "--property=LoadState",
            "--property=ActiveState",
            "--property=SubState",
            "--property=UnitFileState",
            "--no-pager",
        ]
    ) or (len(argv) == 3 and argv[:2] == ["fuser", "--"] and device_name(argv[2]) == argv[2])
    if not allowed:
        raise ValueError("command is outside the read-only whitelist")
    # communicate() buffers command output, so use temporary regular files as
    # bounded readback sinks. These commands normally produce a few hundred bytes.
    import tempfile

    try:
        with tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
            try:
                process = subprocess.run(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=out,
                    stderr=err,
                    timeout=COMMAND_TIMEOUT,
                    check=False,
                    env={**os.environ, "LC_ALL": "C", "SYSTEMD_PAGER": "cat"},
                )
            except subprocess.TimeoutExpired:
                return {"status": "unknown", "reason": "command timeout", "argv": argv}
            out.seek(0)
            err.seek(0)
            stdout, stderr = out.read(READ_LIMIT + 1), err.read(READ_LIMIT + 1)
            if len(stdout) > READ_LIMIT or len(stderr) > READ_LIMIT:
                return {
                    "status": "unknown",
                    "reason": "command output limit exceeded",
                    "argv": argv,
                }
            return {
                "status": "observed",
                "argv": argv,
                "returncode": process.returncode,
                "stdout": stdout.decode("utf-8", errors="replace"),
                "stderr": stderr.decode("utf-8", errors="replace"),
            }
    except OSError as error:
        return {
            "status": "unknown",
            "reason": type(error).__name__ + ": " + str(error),
            "argv": argv,
        }


# 运行中或开机启用的串口登录服务都可能抢占串口；只有明确 inactive/dead 且未启用才通过本项。
def inspect_getty(runner, name):
    unit = "serial-getty@" + PurePosixPath(name).name + ".service"
    row = runner(
        [
            "systemctl",
            "show",
            unit,
            "--property=LoadState",
            "--property=ActiveState",
            "--property=SubState",
            "--property=UnitFileState",
            "--no-pager",
        ]
    )
    row = dict(row)
    fields = dict(line.split("=", 1) for line in row.get("stdout", "").splitlines() if "=" in line)
    row["unit"] = unit
    row["properties"] = fields
    if (
        row.get("status") != "observed"
        or row.get("returncode") != 0
        or fields.get("LoadState") != "loaded"
    ):
        row["status"] = "unknown"
    elif fields.get("ActiveState") in (
        "active",
        "activating",
        "reloading",
        "deactivating",
    ) or fields.get("UnitFileState") in ("enabled", "enabled-runtime"):
        row["status"] = "not_ready"
    elif (
        fields.get("ActiveState") == "inactive"
        and fields.get("SubState") == "dead"
        and fields.get("UnitFileState")
        in ("disabled", "static", "indirect", "masked", "masked-runtime")
    ):
        row["status"] = "observed"
    else:
        row["status"] = "unknown"
    return row


# fuser 只看当前用户可见进程；观察到 PID 即占用，空结果须满足明确退出码且无错误输出才可信。
def inspect_users(runner, name):
    row = dict(runner(["fuser", "--", name]))
    row["scope"] = "Only processes visible to the invoking user; not proof of global exclusivity."
    stdout, stderr = row.get("stdout", "").strip(), row.get("stderr", "").strip()
    if row.get("status") != "observed":
        row["occupancy"] = "unknown"
    elif row.get("returncode") == 0 and re.fullmatch(r"[0-9\s]+", stdout):
        row.update(status="not_ready", occupancy="occupied", pids=[int(x) for x in stdout.split()])
    elif row.get("returncode") == 1 and not stdout and not stderr:
        row.update(status="observed", occupancy="no_users_observed")
    else:
        row.update(status="unknown", occupancy="unknown")
    return row


# 将控制台名称、设备树路径或已知 serial 别名与候选设备对照；无法解释的别名必须返回 unknown。
def console_target(paths, device, dt, target, *, dt_path=False):
    """Return conflict/clear/unknown without assuming an unexplained alias."""
    target = target.split(":" if dt_path else ",", 1)[0]
    if not target:
        return "unknown"
    if target in (device, PurePosixPath(device).name):
        return "conflict"
    if dt_path or re.fullmatch(r"serial[0-9]+", target):
        bound, root = dt.get("bound_node"), dt.get("root")
        resolved = None
        if target.startswith("/") and root and ".." not in PurePosixPath(target).parts:
            try:
                _, resolved, _ = paths.resolve(
                    root + target, scopes=("/sys/firmware/devicetree/base", "/proc/device-tree")
                )
            except (OSError, ValueError):
                pass
        else:
            alias = next((row for row in dt.get("aliases", []) if row["alias"] == target), {})
            resolved = alias.get("resolved_node")
        if resolved and bound:
            return "conflict" if resolved == bound else "clear"
        return "unknown"
    try:
        same = same_device(
            inspect_device(paths, device), inspect_device(paths, device_name(target))
        )
        if same["same_device"] is True:
            return "conflict"
        if same["same_device"] is False:
            return "clear"
    except ValueError:
        pass
    # Kernel console tty names identify a different tty even if that node is not
    # present in /dev. Opaque aliases and arbitrary paths remain unexplained.
    if re.fullmatch(r"(?:tty[A-Za-z]*|hvc|xvc|hvsi|lp)[0-9]+|null|ttynull|netcon", target):
        return "clear"
    return "unknown"


# 分别核对启动参数、活跃控制台与设备树 stdout；任一冲突即不就绪，earlycon 仍需人工解释。
def inspect_console(paths, device, dt):
    sources = {
        "cmdline": text_evidence(paths.read("/proc/cmdline")),
        "proc_consoles": text_evidence(paths.read("/proc/consoles")),
        "sys_console_active": text_evidence(paths.read("/sys/class/tty/console/active")),
    }
    root = dt.get("root")
    stdout = (
        text_evidence(paths.read(root + "/chosen/stdout-path", 4096))
        if root
        else {"status": "unknown", "reason": "Live device tree unavailable"}
    )
    if stdout["status"] == "unknown" and stdout.get("reason", "").startswith("FileNotFoundError:"):
        stdout = text_evidence(paths.read(root + "/chosen/linux,stdout-path", 4096))
    sources["device_tree_stdout"] = stdout
    conflicts, unknown = [], []
    for name, row in sources.items():
        if row["status"] != "observed":
            unknown.append(name)
            continue
        value = row["value"]
        if name == "cmdline":
            targets = [
                part.split("=", 1)[1] for part in value.split() if part.startswith("console=")
            ]
            row["earlycon"] = [
                part for part in value.split() if part == "earlycon" or part.startswith("earlycon=")
            ]
            if row["earlycon"]:
                unknown.append(name + ":earlycon")
        elif name == "proc_consoles":
            targets = [line.split()[0] for line in value.splitlines() if line.strip()]
        elif name == "device_tree_stdout":
            targets = [value]
        else:
            targets = value.split()
        row["targets"] = [
            {
                "target": target,
                "interpretation": console_target(
                    paths, device, dt, target, dt_path=name == "device_tree_stdout"
                ),
            }
            for target in targets
        ]
        if any(target["interpretation"] == "conflict" for target in row["targets"]):
            row["status"] = "not_ready"
            conflicts.append(name)
        elif any(target["interpretation"] == "unknown" for target in row["targets"]):
            row["status"] = "unknown"
            unknown.append(name)
    return {
        "status": "not_ready" if conflicts else "unknown" if unknown else "observed",
        "sources": sources,
        "conflicts": conflicts,
        "unknown_sources": unknown,
        "notice": "cmdline, active kernel consoles and device-tree stdout are checked independently; earlycon needs manual interpretation.",
    }


# 只对默认 ttyS6 核对 serial6、uart6 与 PE0/PE1 的设备树一致性；不泛化为排针/电气验证。
def inspect_default_candidate_mapping(device, dt):
    result = {
        "status": "unknown",
        "applicable": device == "/dev/ttyS6",
        "expected": {
            "device": "/dev/ttyS6",
            "serial_alias": "serial6",
            "pinctrl_function": "uart6",
            "pinctrl_pins": ["PE0", "PE1"],
        },
        "notice": "DT/pinctrl consistency is not physical header or electrical validation.",
    }
    if not result["applicable"]:
        result["reason"] = "Custom device: no physical UART or pin mapping is inferred."
        return result
    bound = dt.get("bound_node")
    alias = next((row for row in dt.get("aliases", []) if row["alias"] == "serial6"), {})
    target = alias.get("resolved_node")
    result["alias_consistency"] = (
        "unknown" if not bound or not target else "observed" if bound == target else "not_ready"
    )
    result["bound_node"] = bound
    result["serial6_node"] = target
    functions, pins, uart6_pins = set(), set(), set()
    for group in dt.get("pinctrl", {}).get("groups", []):
        group_functions, group_pins = set(), set()
        for key in ("function", "allwinner,function"):
            group_functions.update(value.lower() for value in group.get(key, {}).get("value", []))
        for key in ("pins", "allwinner,pins"):
            for value in group.get(key, {}).get("value", []):
                group_pins.update(part.upper() for part in re.split(r"[\s,]+", value) if part)
        functions.update(group_functions)
        pins.update(group_pins)
        if "uart6" in group_functions:
            uart6_pins.update(group_pins)
    result["observed_functions"] = sorted(functions)
    result["observed_pins"] = sorted(pins)
    result["observed_uart6_pins"] = sorted(uart6_pins)
    result["function_consistency"] = (
        "unknown" if not functions else "observed" if "uart6" in functions else "not_ready"
    )
    result["pins_consistency"] = (
        "unknown"
        if not uart6_pins
        else "observed" if {"PE0", "PE1"}.issubset(uart6_pins) else "not_ready"
    )
    statuses = [
        result[key] for key in ("alias_consistency", "function_consistency", "pins_consistency")
    ]
    result["status"] = (
        "not_ready"
        if "not_ready" in statuses
        else "unknown" if "unknown" in statuses else "observed"
    )
    return result


# 只记录启动配置中 UART/overlay/console 相关文本；存在配置文字不等于该 overlay 已实际生效。
def boot_evidence(paths):
    rows = []
    for path in (
        "/boot/orangepiEnv.txt",
        "/boot/armbianEnv.txt",
        "/boot/config.txt",
        "/boot/firmware/config.txt",
        "/boot/extlinux/extlinux.conf",
    ):
        row = text_evidence(paths.read(path))
        if "value" in row:
            row["uart_related_lines"] = [
                line
                for line in row.pop("value").splitlines()
                if re.search(r"uart|overlay|console", line, re.I)
            ]
        row["interpretation"] = (
            "unknown: recorded text is not proof of an active overlay or pin mapping"
        )
        rows.append(row)
    return rows


# 汇总每项本机观测：有明确冲突优先 not_ready，否则缺证据为 unknown；最好的结果也只允许人工复核。
def collect_report(*, paths=None, device="/dev/ttyS6", primary="/dev/ttyS2", runner=None):
    paths = paths or SystemPaths()
    device, primary = device_name(device), device_name(primary)
    # 注入离线文件系统时默认禁用宿主命令，防止夹具中的设备名触发真实 systemctl/fuser 查询。
    if runner is None:
        runner = (
            run_readonly
            if paths.is_host
            else lambda argv: {
                "status": "unknown",
                "argv": argv,
                "reason": "host commands disabled for an injected filesystem",
            }
        )
    root = find_dt_root(paths)
    model = text_evidence(paths.read(root + "/model", 4096)) if root else {"status": "unknown"}
    compatible = (
        strings_evidence(paths.read(root + "/compatible", 4096)) if root else {"status": "unknown"}
    )
    identification = " ".join([model.get("value", "")] + compatible.get("value", []))
    confirmed = bool(
        re.search(r"orange\s*pi[\s,-]*zero[\s-]*3w|orangepi-zero3w", identification, re.I)
    )
    board_status = "OBSERVED_ORANGEPI_ZERO3W_IDENTITY" if confirmed else "NOT_CONFIRMED_ON_ORANGEPI"
    candidate, primary_row = inspect_device(paths, device), inspect_device(paths, primary)
    distinct = same_device(candidate, primary_row)
    binding = inspect_binding(paths, device)
    dt = inspect_dt(paths, root, binding)
    checks = {
        "board_identity": {
            "status": "observed" if confirmed else "unknown",
            "result": board_status,
        },
        "candidate_character_device": candidate,
        "primary_character_device": primary_row,
        "distinct_from_primary": distinct,
        "candidate_driver_binding": binding,
        "candidate_device_tree": dt,
        "candidate_pinctrl": dt.get("pinctrl", {"status": "unknown"}),
        "default_candidate_mapping": inspect_default_candidate_mapping(device, dt),
        "kernel_console": inspect_console(paths, device, dt),
        "serial_getty": inspect_getty(runner, device),
        "visible_process_users": inspect_users(runner, device),
    }
    next_steps = []
    for key, row in checks.items():
        if row["status"] != "observed":
            next_steps.append(
                {
                    "check": key,
                    "status": row["status"],
                    "action": "Resolve this missing or conflicting evidence by manual review before enabling the candidate UART.",
                }
            )
    # 物理接线矛盾和端到端电气/数据链路始终单列未决项，不能由 Linux 元数据检查代替。
    next_steps.extend(
        [
            {
                "check": "physical_header_and_peer",
                "status": "unknown",
                "action": "Resolve the conflict between reported primary FC01 RX6/TX6 wiring and supplied "
                "NSH /dev/ttyS3 evidence mapped to UART4 by local source. Preserve the working link; "
                "candidate UART4 availability is unresolved and must not be enabled.",
            },
            {
                "check": "electrical_and_bidirectional_link",
                "status": "unknown",
                "action": "This passive report does not validate voltage, wiring, baud, data transfer, or the flight-controller endpoint.",
            },
        ]
    )
    statuses = [row["status"] for row in checks.values()]
    summary = (
        "not_ready"
        if "not_ready" in statuses
        else "unknown" if "unknown" in statuses else "eligible_for_manual_review"
    )
    tty_drivers = text_evidence(paths.read("/proc/tty/drivers"))
    tty_drivers["notice"] = "Registered tty drivers are not proof of this tty's bound driver."
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "summary": summary,
        "summary_scope": "LOCAL_LINUX_CHECKS_ONLY_NOT_END_TO_END_LINK_READINESS",
        "hardware_validation": "NOT_PERFORMED",
        "board_confirmation": board_status,
        "DECLARED_USER_MAPPING": DECLARED_USER_MAPPING,
        "FC01_SOURCE_REFERENCE": FC01_SOURCE_REFERENCE,
        "USER_REPORTED_PROGRESS": USER_REPORTED_PROGRESS,
        "USER_PROVIDED_FC01_RUNTIME": USER_PROVIDED_FC01_RUNTIME,
        "FC01_EVIDENCE_RECONCILIATION": reconcile_fc01_evidence(
            DECLARED_USER_MAPPING, USER_PROVIDED_FC01_RUNTIME, FC01_SOURCE_REFERENCE
        ),
        "ACTUAL_LOCAL_OBSERVATION": {
            "evidence_class": "ACTUAL_LOCAL_OBSERVATION",
            "filesystem_context": (
                "live_host" if paths.is_host else "injected_filesystem_not_hardware_evidence"
            ),
            "collector_host_machine": os.uname().machine,
            "requested_device": device,
            "requested_primary": primary,
            "os_release": text_evidence(paths.read("/etc/os-release", 16384)),
            "kernel_version": text_evidence(paths.read("/proc/version", 16384)),
            "board_model": model,
            "board_compatible": compatible,
            "checks": checks,
            "registered_tty_drivers": tty_drivers,
            "boot_configuration": boot_evidence(paths),
        },
        "next_steps": next_steps,
        "scope": [
            "No serial device was opened; no termios, UART writes, agent, or service changes.",
            "No overlay, boot configuration, firmware, flight parameter, or existing primary link changes.",
            "The highest possible outcome is eligible_for_manual_review, never hardware validated.",
            "An x86/other host cannot establish OrangePi UART availability.",
            "Dated user evidence and source references do not describe this collector's current observations.",
            "fuser reports only processes visible to this user at collection time.",
        ],
    }


# 输出必须是全新私有目录；JSON 保存结构化证据，文本给出结论边界与下一步复核项。
def write_report(report, output):
    output = Path(output)
    # mkdir without exist_ok makes an existing path (including symlinks) an error.
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    (output / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    lines = [
        "UART passive preflight",
        "",
        "Summary: " + report["summary"],
        "Board: " + report["board_confirmation"],
        "Hardware validation: NOT_PERFORMED",
        "",
        "User wiring hypotheses, reported progress, source references and actual local observations are separate in report.json.",
        "Summary covers local Linux checks only, not end-to-end link readiness.",
        "FC01 dated evidence: " + report["FC01_EVIDENCE_RECONCILIATION"]["status"],
        "Candidate FC01 availability: "
        + report["FC01_EVIDENCE_RECONCILIATION"]["candidate_availability"],
        "PX4 logical TELEM4/104 is not USART6/URT6/6.",
        "No UART device was opened or configured.",
        "",
        "Evidence requiring further review:",
    ]
    lines.extend(
        "- " + row["check"] + " [" + row["status"] + "]: " + row["action"]
        for row in report["next_steps"]
    )
    (output / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


# 命令行只选择候选、主设备与新输出目录；调用采集并显示报告位置，不启动 Agent 或导航服务。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new report directory; must not already exist")
    parser.add_argument(
        "--device", default="/dev/ttyS6", help="candidate tty basename or /dev path"
    )
    parser.add_argument(
        "--primary", default="/dev/ttyS2", help="existing primary tty basename or /dev path"
    )
    args = parser.parse_args(argv)
    try:
        device, primary = device_name(args.device), device_name(args.primary)
        output = args.output or Path(
            "uart_preflight_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        )
        if output.exists() or output.is_symlink():
            raise ValueError("output directory must be new")
        report = collect_report(device=device, primary=primary)
        write_report(report, output)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(str(output.resolve()))
    print(
        report["summary"]
        + "; "
        + report["board_confirmation"]
        + "; hardware validation NOT_PERFORMED"
    )
    comparison = report["FC01_EVIDENCE_RECONCILIATION"]
    print(
        "FC01 dated evidence: "
        + comparison["status"]
        + "; candidate: "
        + comparison["candidate_availability"]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
