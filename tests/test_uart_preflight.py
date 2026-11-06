"""The passive audit never opens a UART or changes the inspected system."""
import copy
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import uart_preflight as audit


ROOT = Path(__file__).resolve().parents[1]


class Fixture:
    def __init__(self, root):
        self.root = root
        self.dt = "/sys/firmware/devicetree/base"
        self.node = self.dt + "/soc/serial@2506000"
        self.platform = "/sys/devices/platform/soc/serial"
        self.write("/etc/os-release", 'NAME="fixture Linux"\n')
        self.write("/proc/version", "Linux fixture\n")
        self.write("/proc/cmdline", "root=/dev/mmcblk0 console=ttyS2,115200\n")
        self.write("/proc/consoles", "ttyS2                -W- (EC p a)    4:66\n")
        self.write("/sys/class/tty/console/active", "ttyS2\n")
        self.write("/proc/tty/drivers", "serial /dev/ttyS 4 64-111 serial\n")
        self.write(self.dt + "/model", b"Orange Pi Zero3W V1.2\x00")
        self.write(self.dt + "/compatible", b"xunlong,orangepi-zero3w\x00allwinner,sun60i-a733\x00")
        self.write(self.dt + "/aliases/serial6", b"/soc/serial@2506000\x00")
        self.write(self.dt + "/aliases/serial2", b"/soc/serial@2500800\x00")
        self.write(self.dt + "/soc/serial@2500800/status", b"okay\x00")
        self.write(self.dt + "/chosen/stdout-path", b"serial2:115200n8\x00")
        self.write(self.node + "/status", b"okay\x00")
        self.write(self.node + "/compatible", b"snps,dw-apb-uart\x00")
        self.write(self.node + "/pinctrl-names", b"default\x00")
        self.write(self.node + "/pinctrl-0", (17).to_bytes(4, "big"))
        group = self.dt + "/soc/pinctrl/uart6-pins"
        self.write(group + "/phandle", (17).to_bytes(4, "big"))
        self.write(group + "/function", b"uart6\x00")
        self.write(group + "/pins", b"PE0\x00PE1\x00")
        (root / (self.platform + "/tty/ttyS6").lstrip("/")).mkdir(parents=True)
        (root / "sys/bus/platform/drivers/serial8250").mkdir(parents=True)
        self.link("/sys/class/tty/ttyS6/device", self.platform + "/tty/ttyS6")
        self.link(self.platform + "/driver", "/sys/bus/platform/drivers/serial8250")
        self.link(self.platform + "/of_node", self.node)
        self.write("/dev/ttyS6", "do not open candidate")
        self.write("/dev/ttyS2", "do not open primary")
        self.write("/boot/orangepiEnv.txt", "overlays=uart6\n$(touch MUST_NOT_EXIST)\n")
        self.commands = []
        self.fuser = {"status": "observed", "returncode": 1, "stdout": "", "stderr": ""}
        self.getty = {"status": "observed", "returncode": 0,
                      "stdout": "LoadState=loaded\nActiveState=inactive\nSubState=dead\nUnitFileState=disabled\n",
                      "stderr": ""}

    def write(self, logical, data):
        path = self.root / logical.lstrip("/")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data if isinstance(data, bytes) else data.encode())

    def link(self, logical, target):
        path = self.root / logical.lstrip("/")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.symlink_to(target)

    def runner(self, argv):
        self.commands.append(argv)
        row = self.fuser if argv[0] == "fuser" else self.getty
        return dict(copy.deepcopy(row), argv=argv)

    def report(self, **kwargs):
        # Unprivileged tests cannot mknod: synthesize only stat metadata for the
        # two fixture nodes. Their contents still must never be opened.
        original = Path.stat
        dev = self.root / "dev"
        def device_stat(path, *args, **options):
            info = original(path, *args, **options)
            if path.parent == dev and path.name in ("ttyS6", "ttyS2"):
                return SimpleNamespace(st_mode=stat.S_IFCHR | 0o660,
                                       st_rdev=os.makedev(4, 70 if path.name == "ttyS6" else 66),
                                       st_dev=info.st_dev, st_ino=info.st_ino)
            return info
        with patch.object(Path, "stat", device_stat):
            return audit.collect_report(paths=audit.SystemPaths(self.root), runner=self.runner, **kwargs)


class UartPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.fixture = Fixture(self.root)

    def checks(self, report):
        return report["ACTUAL_LOCAL_OBSERVATION"]["checks"]

    def test_fake_orangepi_mapping_is_only_eligible_for_manual_review(self):
        report = self.fixture.report()
        self.assertEqual(report["summary"], "eligible_for_manual_review")
        self.assertEqual(report["hardware_validation"], "NOT_PERFORMED")
        self.assertEqual(report["DECLARED_USER_MAPPING"]["primary"]["change_policy"], "DO_NOT_CHANGE")
        self.assertEqual(report["DECLARED_USER_MAPPING"]["candidate"]["tx_pin"], 24)
        self.assertEqual(report["DECLARED_USER_MAPPING"]["candidate"]["rx_pin"], 23)
        self.assertEqual(report["DECLARED_USER_MAPPING"]["candidate"]["ground_pin"], 25)
        checks = self.checks(report)
        self.assertEqual(checks["candidate_driver_binding"]["driver"]["path"], "/sys/bus/platform/drivers/serial8250")
        self.assertIn("serial6", [row["alias"] for row in checks["candidate_device_tree"]["aliases"]])
        self.assertEqual(checks["candidate_pinctrl"]["groups"][0]["function"]["value"], ["uart6"])
        self.assertEqual(checks["default_candidate_mapping"]["status"], "observed")
        self.assertTrue(any(row["check"] == "physical_header_and_peer" for row in report["next_steps"]))

    def test_corrected_fc01_hypothesis_keeps_orangepi_devices_and_pins(self):
        report = self.fixture.report()
        declaration = report["DECLARED_USER_MAPPING"]
        self.assertEqual(declaration["evidence_class"], "USER_REPORTED_PHYSICAL_CONNECTION")
        primary, candidate = declaration["primary"], declaration["candidate"]
        self.assertEqual((primary["device"], primary["uart"], primary["tx_pin"], primary["rx_pin"], primary["baud"]),
                         ("/dev/ttyS2", "UART2", 11, 13, 115200))
        self.assertEqual((candidate["device"], candidate["uart"], candidate["tx_pin"], candidate["rx_pin"]),
                         ("/dev/ttyS6", "UART6", 24, 23))
        self.assertEqual((primary["peer_hardware_uart_claim"], primary["peer_physical_connector_claim"]),
                         ("USART6", None))
        self.assertEqual((candidate["peer_hardware_uart_claim"], candidate["peer_physical_connector_claim"]),
                         ("UART4", "TELEM3"))
        self.assertEqual(primary["peer_verification"], "USER_REPORTED_PHYSICAL_CONNECTION")
        self.assertEqual(candidate["peer_verification"], "PENDING_ON_DEVICE")
        observation = report["ACTUAL_LOCAL_OBSERVATION"]
        self.assertEqual((observation["requested_primary"], observation["requested_device"]),
                         ("/dev/ttyS2", "/dev/ttyS6"))
        self.assertEqual(report["hardware_validation"], "NOT_PERFORMED")

    def test_physical_telem4_cannot_alias_px4_logical_telem4(self):
        report = self.fixture.report()
        reference = report["FC01_SOURCE_REFERENCE"]
        self.assertEqual(reference["evidence_class"], "LOCAL_FIRMWARE_SOURCE_REFERENCE")
        self.assertEqual(reference["on_device_firmware_match"], "NOT_VERIFIED")
        ports = {row["stm32_hardware_uart"]: row for row in reference["ports"]}
        observed = [(ports[uart]["px4_logical_port"], ports[uart]["px4_selector"],
                     ports[uart]["nuttx_device"], ports[uart]["baud_parameter"])
                    for uart in ("UART4", "USART6", "UART7")]
        self.assertEqual(observed, [("TELEM3", 103, "/dev/ttyS3", "SER_TEL3_BAUD"),
                                    ("URT6", 6, "/dev/ttyS5", "SER_URT6_BAUD"),
                                    ("TELEM4", 104, "/dev/ttyS6", "SER_TEL4_BAUD")])
        self.assertEqual(len({row["px4_selector"] for row in ports.values()}), 3)
        self.assertNotEqual(ports["USART6"]["nuttx_device"], report["ACTUAL_LOCAL_OBSERVATION"]["requested_device"])
        self.assertTrue(all(cmd[0] in ("systemctl", "fuser") for cmd in self.fixture.commands))

    def test_user_reported_progress_does_not_override_current_observation(self):
        self.fixture.write(self.fixture.dt + "/model", "Fixture desktop")
        self.fixture.write(self.fixture.dt + "/compatible", "fixture,desktop")
        (self.root / "dev/ttyS6").unlink()
        report = self.fixture.report()
        progress = report["USER_REPORTED_PROGRESS"]
        self.assertEqual(progress["evidence_class"], "USER_REPORTED_PROGRESS")
        self.assertEqual(progress["orangepi"]["candidate_device_registered"], "/dev/ttyS6")
        self.assertEqual(progress["orangepi"]["candidate_driver"], "uart-ng")
        self.assertEqual(progress["orangepi"]["removed_conflicting_overlay"], "spi3-cs0-cs1-spidev")
        self.assertEqual(report["board_confirmation"], "NOT_CONFIRMED_ON_ORANGEPI")
        self.assertEqual(report["summary"], "unknown")
        self.assertEqual(self.checks(report)["candidate_character_device"]["status"], "unknown")
        self.assertEqual(report["hardware_validation"], "NOT_PERFORMED")

    def test_user_nsh_evidence_confirms_device_not_physical_uart(self):
        report = self.fixture.report()
        evidence = report["USER_PROVIDED_FC01_RUNTIME"]
        self.assertEqual(evidence["evidence_class"], "USER_PROVIDED_NSH_OUTPUT")
        self.assertEqual(evidence["capture_time"], "UNKNOWN")
        self.assertEqual((evidence["UXRCE_DDS_CFG"], evidence["startup_device"], evidence["startup_baud"]),
                         (103, "/dev/ttyS3", 115200))
        self.assertNotIn("USER_PROVIDED_FC01_RUNTIME", report["ACTUAL_LOCAL_OBSERVATION"])
        self.assertEqual(report["DECLARED_USER_MAPPING"]["candidate"]["peer_verification"], "PENDING_ON_DEVICE")
        self.assertEqual(report["hardware_validation"], "NOT_PERFORMED")
        self.assertTrue(all(cmd[0] in ("systemctl", "fuser") for cmd in self.fixture.commands))

    def test_linux_eligible_cannot_override_conflicting_fc01_evidence(self):
        report = self.fixture.report()
        comparison = report["FC01_EVIDENCE_RECONCILIATION"]
        self.assertEqual(report["summary"], "eligible_for_manual_review")
        self.assertEqual(report["summary_scope"], "LOCAL_LINUX_CHECKS_ONLY_NOT_END_TO_END_LINK_READINESS")
        self.assertEqual(comparison["status"], "CONFLICT")
        self.assertEqual(comparison["source_mapped_runtime_hardware_uart"], "UART4")
        self.assertEqual(comparison["reported_primary_hardware_uart"], "USART6")
        self.assertTrue(comparison["candidate_matches_source_mapped_runtime"])
        self.assertEqual(comparison["candidate_availability"], "UNRESOLVED_DO_NOT_ENABLE")

    def test_fc01_comparison_resolves_selector_and_device_without_linux_number_inference(self):
        runtime = copy.deepcopy(audit.USER_PROVIDED_FC01_RUNTIME)
        for selector, device, hardware, status in (
                (103, "/dev/ttyS3", "UART4", "CONFLICT"),
                (6, "/dev/ttyS5", "USART6", "CONSISTENT_DATED_EVIDENCE_ONLY"),
                (104, "/dev/ttyS6", "UART7", "CONFLICT")):
            with self.subTest(selector=selector):
                runtime.update(UXRCE_DDS_CFG=selector, startup_device=device)
                row = audit.reconcile_fc01_evidence(audit.DECLARED_USER_MAPPING, runtime, audit.FC01_SOURCE_REFERENCE)
                self.assertEqual(row["source_mapped_runtime_hardware_uart"], hardware)
                self.assertEqual(row["status"], status)
                self.assertEqual(row["candidate_availability"], "UNRESOLVED_DO_NOT_ENABLE")

    def test_missing_or_mismatched_runtime_evidence_cannot_resolve_fc01_conflict(self):
        runtime = copy.deepcopy(audit.USER_PROVIDED_FC01_RUNTIME)
        runtime["startup_device"] = "/dev/ttyS5"
        row = audit.reconcile_fc01_evidence(audit.DECLARED_USER_MAPPING, runtime, audit.FC01_SOURCE_REFERENCE)
        self.assertEqual(row["status"], "CONFLICT")
        self.assertIsNone(row["source_mapped_runtime_hardware_uart"])
        runtime.pop("UXRCE_DDS_CFG")
        row = audit.reconcile_fc01_evidence(audit.DECLARED_USER_MAPPING, runtime, audit.FC01_SOURCE_REFERENCE)
        self.assertEqual(row["status"], "UNKNOWN")
        self.assertEqual(row["candidate_availability"], "UNRESOLVED_DO_NOT_ENABLE")

    def test_device_type_missing_and_primary_missing_cannot_pass(self):
        raw = audit.collect_report(paths=audit.SystemPaths(self.root), runner=self.fixture.runner)
        self.assertEqual(self.checks(raw)["candidate_character_device"]["status"], "not_ready")
        (self.root / "dev/ttyS6").unlink()
        missing = self.fixture.report()
        self.assertEqual(self.checks(missing)["candidate_character_device"]["status"], "unknown")
        self.assertNotEqual(missing["summary"], "eligible_for_manual_review")
        (self.root / "dev/ttyS2").unlink()
        self.assertEqual(self.checks(self.fixture.report())["primary_character_device"]["status"], "unknown")

    def test_missing_live_device_tree_cannot_claim_orangepi(self):
        other = self.root / "empty-root"
        other.mkdir()
        report = audit.collect_report(paths=audit.SystemPaths(other), runner=self.fixture.runner)
        self.assertEqual(report["board_confirmation"], "NOT_CONFIRMED_ON_ORANGEPI")
        self.assertEqual(report["summary"], "unknown")
        self.assertEqual(self.checks(report)["candidate_device_tree"]["status"], "unknown")

    def test_disabled_bound_node_and_disabled_parent_are_not_ready(self):
        for target in (self.fixture.node, self.fixture.dt + "/soc"):
            with self.subTest(target=target):
                self.fixture.write(target + "/status", b"disabled\x00")
                report = self.fixture.report()
                self.assertEqual(self.checks(report)["candidate_device_tree"]["status"], "not_ready")
                self.assertEqual(report["summary"], "not_ready")
                self.fixture.write(target + "/status", b"okay\x00")

    def test_missing_or_permission_denied_dt_status_is_unknown(self):
        target = self.root / (self.fixture.node + "/status").lstrip("/")
        target.unlink()
        self.assertEqual(self.checks(self.fixture.report())["candidate_device_tree"]["status"], "unknown")
        self.fixture.write(self.fixture.node + "/status", "okay")
        original = os.open
        def denied(path, *args, **kwargs):
            if Path(path) == target:
                raise PermissionError("fixture access denied")
            return original(path, *args, **kwargs)
        with patch.object(os, "open", denied):
            self.assertEqual(self.checks(self.fixture.report())["candidate_device_tree"]["status"], "unknown")

    def test_serial_alias_is_not_a_binding_and_registered_driver_is_not_proof(self):
        (self.root / (self.fixture.platform + "/driver").lstrip("/")).unlink()
        report = self.fixture.report()
        self.assertEqual(self.checks(report)["candidate_driver_binding"]["status"], "unknown")
        self.assertIn("not proof", report["ACTUAL_LOCAL_OBSERVATION"]["registered_tty_drivers"]["notice"])
        (self.root / (self.fixture.platform + "/of_node").lstrip("/")).unlink()
        self.assertEqual(self.checks(self.fixture.report())["candidate_device_tree"]["status"], "unknown")

    def test_missing_pinctrl_function_or_unresolved_phandle_is_unknown(self):
        target = self.root / (self.fixture.dt + "/soc/pinctrl/uart6-pins/function").lstrip("/")
        target.unlink()
        self.assertEqual(self.checks(self.fixture.report())["candidate_pinctrl"]["status"], "unknown")
        self.fixture.write(self.fixture.node + "/pinctrl-0", (999).to_bytes(4, "big"))
        self.assertEqual(self.checks(self.fixture.report())["candidate_pinctrl"]["status"], "unknown")

    def test_fuser_exit_codes_and_timeout_are_interpreted_conservatively(self):
        cases = [
            ({"status": "observed", "returncode": 0, "stdout": " 123 456 ", "stderr": "/dev/ttyS6:"}, "not_ready", "occupied"),
            ({"status": "observed", "returncode": 1, "stdout": "", "stderr": ""}, "observed", "no_users_observed"),
            ({"status": "observed", "returncode": 1, "stdout": "", "stderr": "Permission denied"}, "unknown", "unknown"),
            ({"status": "observed", "returncode": 2, "stdout": "", "stderr": "failure"}, "unknown", "unknown"),
            ({"status": "observed", "returncode": 0, "stdout": "", "stderr": ""}, "unknown", "unknown"),
            ({"status": "unknown", "reason": "command timeout"}, "unknown", "unknown"),
        ]
        for raw, status, occupancy in cases:
            with self.subTest(raw=raw):
                self.fixture.fuser = raw
                row = self.checks(self.fixture.report())["visible_process_users"]
                self.assertEqual((row["status"], row["occupancy"]), (status, occupancy))

    def test_console_candidate_alias_and_earlycon(self):
        for cmdline, expected in (("console=ttyS6,115200", "not_ready"),
                                  ("console=serial6,115200", "not_ready"),
                                  ("console=ttyS2,115200", "observed"),
                                  ("earlycon=uart8250,mmio,0x2506000", "unknown")):
            with self.subTest(cmdline=cmdline):
                self.fixture.write("/proc/cmdline", cmdline)
                self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], expected)

    def test_registered_and_active_consoles_detect_conflict_without_cmdline(self):
        for path, conflict, clear in (
                ("/proc/consoles", "ttyS6 -W- (EC p a) 4:70\n", "ttyS2 -W- (EC p a) 4:66\n"),
                ("/sys/class/tty/console/active", "tty0 ttyS6\n", "ttyS2\n")):
            with self.subTest(path=path):
                self.fixture.write(path, conflict)
                row = self.checks(self.fixture.report())["kernel_console"]
                self.assertEqual(row["status"], "not_ready")
                self.assertEqual(row["sources"]["cmdline"]["status"], "observed")
                self.fixture.write(path, clear)

    def test_device_tree_stdout_alias_absolute_path_and_linux_fallback(self):
        stdout = self.fixture.dt + "/chosen/stdout-path"
        for target in ("serial6:115200n8", "/soc/serial@2506000:115200n8"):
            with self.subTest(target=target):
                self.fixture.write(stdout, target)
                self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], "not_ready")
        (self.root / stdout.lstrip("/")).unlink()
        self.fixture.write(self.fixture.dt + "/chosen/linux,stdout-path", "serial6:115200n8")
        row = self.checks(self.fixture.report())["kernel_console"]
        self.assertEqual(row["status"], "not_ready")
        self.assertTrue(row["sources"]["device_tree_stdout"]["path"].endswith("linux,stdout-path"))

    def test_missing_uninterpretable_or_denied_console_evidence_is_unknown(self):
        for path, restore in (("/proc/consoles", "ttyS2 -W- (EC p a) 4:66\n"),
                              ("/sys/class/tty/console/active", "ttyS2\n"),
                              (self.fixture.dt + "/chosen/stdout-path", "serial2:115200n8")):
            with self.subTest(path=path):
                target = self.root / path.lstrip("/")
                target.unlink()
                self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], "unknown")
                self.fixture.write(path, restore)
        self.fixture.write(self.fixture.dt + "/chosen/stdout-path", "unresolved_alias:115200n8")
        self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], "unknown")
        self.fixture.write(self.fixture.dt + "/chosen/stdout-path", "serial2:115200n8")
        original = os.open
        denied_path = self.root / (self.fixture.dt + "/chosen/stdout-path").lstrip("/")
        # A denied primary property must not silently fall back to a different one.
        self.fixture.write(self.fixture.dt + "/chosen/linux,stdout-path", "serial2:115200n8")
        def denied(path, *args, **kwargs):
            if Path(path) == denied_path:
                raise PermissionError("fixture console denied")
            return original(path, *args, **kwargs)
        with patch.object(os, "open", denied):
            row = self.checks(self.fixture.report())["kernel_console"]
        self.assertEqual(row["status"], "unknown")
        self.assertTrue(row["sources"]["device_tree_stdout"]["path"].endswith("/stdout-path"))
        self.fixture.write("/proc/cmdline", "console=ttyS6,115200")
        (self.root / "proc/consoles").unlink()
        self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], "not_ready")

    def test_default_mapping_observed_alias_function_or_pin_mismatch_is_not_ready(self):
        group = self.fixture.dt + "/soc/pinctrl/uart6-pins"
        for path, mismatch, restore in (
                (self.fixture.dt + "/aliases/serial6", "/soc/serial@2500800", "/soc/serial@2506000"),
                (group + "/function", "uart2", "uart6"),
                (group + "/pins", b"PB0\x00PB1\x00", b"PE0\x00PE1\x00")):
            with self.subTest(path=path):
                self.fixture.write(path, mismatch)
                report = self.fixture.report()
                self.assertEqual(self.checks(report)["default_candidate_mapping"]["status"], "not_ready")
                self.assertEqual(report["summary"], "not_ready")
                self.fixture.write(path, restore)

    def test_default_mapping_missing_evidence_unknown_and_custom_device_not_inferred(self):
        alias = self.root / (self.fixture.dt + "/aliases/serial6").lstrip("/")
        alias.unlink()
        report = self.fixture.report()
        self.assertEqual(self.checks(report)["default_candidate_mapping"]["status"], "unknown")
        self.fixture.link("/dev/candidateAlias", "/dev/ttyS6")
        custom = self.checks(self.fixture.report(device="candidateAlias"))["default_candidate_mapping"]
        self.assertFalse(custom["applicable"])
        self.assertEqual(custom["status"], "unknown")
        self.assertNotIn("observed_pins", custom)

    def test_expected_pins_in_a_different_uart_group_do_not_confirm_uart6(self):
        group = self.fixture.dt + "/soc/pinctrl/uart6-pins"
        self.fixture.write(group + "/pins", b"PB0\x00PB1\x00")
        other = self.fixture.dt + "/soc/pinctrl/other-uart-pins"
        self.fixture.write(other + "/phandle", (18).to_bytes(4, "big"))
        self.fixture.write(other + "/function", b"uart2\x00")
        self.fixture.write(other + "/pins", b"PE0\x00PE1\x00")
        self.fixture.write(self.fixture.node + "/pinctrl-0", (17).to_bytes(4, "big") + (18).to_bytes(4, "big"))
        row = self.checks(self.fixture.report())["default_candidate_mapping"]
        self.assertEqual(row["status"], "not_ready")
        self.assertEqual(row["observed_uart6_pins"], ["PB0", "PB1"])

    def test_same_device_alias_and_identical_device_number_conflict(self):
        self.fixture.link("/dev/candidateAlias", "/dev/ttyS2")
        report = self.fixture.report(device="candidateAlias")
        self.assertTrue(self.checks(report)["distinct_from_primary"]["same_device"])
        self.assertEqual(report["summary"], "not_ready")
        candidate = {"identity": [1, 10], "type": "character_device", "major": 4, "minor": 66}
        primary = dict(candidate, identity=[1, 11])
        self.assertEqual(audit.same_device(candidate, primary)["status"], "not_ready")

    def test_getty_active_enabled_missing_and_command_failure(self):
        for fields, code, expected in (
            ("LoadState=loaded\nActiveState=active\nSubState=running\nUnitFileState=disabled", 0, "not_ready"),
            ("LoadState=loaded\nActiveState=inactive\nSubState=dead\nUnitFileState=enabled", 0, "not_ready"),
            ("LoadState=not-found\nActiveState=inactive", 0, "unknown"),
            ("LoadState=loaded\nActiveState=inactive\nSubState=dead", 0, "unknown"),
            ("", 1, "unknown")):
            with self.subTest(fields=fields):
                self.fixture.getty.update(stdout=fields, returncode=code)
                self.assertEqual(self.checks(self.fixture.report())["serial_getty"]["status"], expected)

    def test_bounded_reads_and_abnormal_symlinks_never_open_a_device(self):
        self.fixture.write("/proc/cmdline", "x" * (audit.READ_LIMIT + 1))
        self.assertEqual(self.checks(self.fixture.report())["kernel_console"]["status"], "unknown")
        status_path = self.root / (self.fixture.node + "/status").lstrip("/")
        status_path.unlink()
        status_path.symlink_to("/dev/ttyS6")
        original = os.open
        opened = []
        def guarded(path, *args, **kwargs):
            opened.append(str(path))
            self.assertFalse(str(path).startswith(str(self.root / "dev")))
            return original(path, *args, **kwargs)
        with patch.object(os, "open", guarded):
            report = self.fixture.report()
        self.assertEqual(self.checks(report)["candidate_device_tree"]["status"], "unknown")
        self.assertFalse((self.root / "MUST_NOT_EXIST").exists())

    def test_inspection_has_no_filesystem_or_process_mutations(self):
        def snapshot():
            return {str(path.relative_to(self.root)):
                    ("symlink", os.readlink(path)) if path.is_symlink() else
                    ("file", path.read_bytes()) for path in self.root.rglob("*") if path.is_file() or path.is_symlink()}
        before = snapshot()
        with patch.object(audit.subprocess, "run", side_effect=AssertionError("unexpected process")):
            self.fixture.report()
        self.assertEqual(snapshot(), before)
        self.assertEqual({tuple(cmd[:2]) for cmd in self.fixture.commands}, {("systemctl", "show"), ("fuser", "--")})
        self.assertTrue(all("sudo" not in cmd and "start" not in cmd for cmd in self.fixture.commands))

    def test_injected_mounts_rebase_absolute_links_and_disable_host_commands(self):
        paths = audit.SystemPaths(self.root / "unused", sys_root=self.root / "sys",
                                  proc_root=self.root / "proc", dev_root=self.root / "dev")
        self.assertEqual(audit.inspect_binding(paths, "/dev/ttyS6")["status"], "observed")
        with patch.object(audit, "run_readonly", side_effect=AssertionError("host command")):
            report = audit.collect_report(paths=paths)
        self.assertEqual(self.checks(report)["visible_process_users"]["status"], "unknown")

    def test_output_must_be_new_and_cli_rejects_existing_before_collection(self):
        output = self.root / "report"
        audit.write_report(self.fixture.report(), output)
        content = (output / "report.json").read_bytes()
        with self.assertRaises(FileExistsError):
            audit.write_report(self.fixture.report(), output)
        self.assertEqual((output / "report.json").read_bytes(), content)
        with patch.object(audit, "collect_report", side_effect=AssertionError("collected before output check")), \
                self.assertRaises(SystemExit) as caught:
            audit.main(["--output", str(output)])
        self.assertEqual(caught.exception.code, 2)

    def test_device_arguments_and_command_whitelist_reject_unsafe_forms(self):
        for name in ("/tmp/ttyS6", "/dev/../etc/passwd", "/dev/ttyS6;touch x", "/dev/tty S6", "/dev/./ttyS6"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                audit.device_name(name)
        for command in (["systemctl", "start", "serial-getty@ttyS6.service"],
                        ["sudo", "fuser", "/dev/ttyS6"], ["fuser", "-k", "/dev/ttyS6"],
                        ["fuser", "--", "/etc/passwd"]):
            with self.subTest(command=command), self.assertRaises(ValueError):
                audit.run_readonly(command)

    def test_readonly_commands_have_timeout_and_no_shell(self):
        command = ["fuser", "--", "/dev/ttyS6"]
        with patch.object(audit.subprocess, "run", return_value=SimpleNamespace(returncode=1)) as run:
            self.assertEqual(audit.run_readonly(command)["returncode"], 1)
        kwargs = run.call_args.kwargs
        self.assertEqual(kwargs["timeout"], audit.COMMAND_TIMEOUT)
        self.assertNotIn("shell", kwargs)
        self.assertEqual(kwargs["stdin"], subprocess.DEVNULL)
        with patch.object(audit.subprocess, "run", side_effect=subprocess.TimeoutExpired(command, 4)):
            self.assertEqual(audit.run_readonly(command)["status"], "unknown")

    @unittest.skipUnless(os.uname().machine in ("x86_64", "i386", "i686"), "x86 host smoke")
    def test_real_host_smoke_is_not_confirmed_on_orangepi(self):
        output = self.root / "host-smoke"
        process = subprocess.run([sys.executable, str(ROOT / "uart_preflight.py"), "--output", str(output)],
                                 capture_output=True, text=True, timeout=20, check=False)
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((output / "report.json").read_text())
        self.assertEqual(report["board_confirmation"], "NOT_CONFIRMED_ON_ORANGEPI")
        self.assertNotEqual(report["summary"], "eligible_for_manual_review")
        self.assertEqual(report["hardware_validation"], "NOT_PERFORMED")
        self.assertTrue((output / "report.txt").is_file())
        self.assertIn("FC01 dated evidence: CONFLICT; candidate: UNRESOLVED_DO_NOT_ENABLE", process.stdout)


if __name__ == "__main__":
    unittest.main()
