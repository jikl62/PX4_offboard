# 2026-11-06 UART 映射审计验证记录

本记录对应分支 `dev/uart-topology-20261105`、基线 `50a2293`。开发机为 x86；没有 SSH 连接香橙派或 FC01，也没有修改现场服务、飞控参数、固件或接线。用户提供的 NSH/MAVLink 输出与 Orange Pi 终端结果按“用户提供证据”保存，未冒充本机观测。

## 回归结果

| 检查 | 结果 | 范围 |
|---|---|---|
| `python3 -m unittest discover -s tests -v` | **213/213 通过，30.267 s**；日志 `/tmp/fc01_20261106_uart_regression.log` | 原有回归与 UART 审计测试；不等于实机串口或飞行验证 |
| `python3 -m unittest discover -s tests -p 'test_uart_preflight.py' -v` | **31/31 通过，0.296 s** | 用户线束声明、103→S3 源码映射冲突、6→S5/104→S6 区分、候选禁止启用、只读边界及异常输入 |
| `python3 -m py_compile uart_preflight.py tests/test_uart_preflight.py` | 通过 | Python 语法 |
| 证据 JSON 解析 | `20261106_uart_mapping_audit.json`、历史 `20261105_uart_source_audit.json` 均通过 `json.tool` | 证据格式 |
| 本地 PX4/NuttX 来源指纹 | **19/19 匹配** | `default.px4board`、`board.h`、NuttX `.config`、`defconfig`、串口驱动、pinmap、启动脚本和 uXRCE 文件未在审计期间变化；不证明现场镜像字节一致 |
| 开发机实际 smoke | `/tmp/fc01_uart_host_20261106_01/report.json`：`unknown`，`NOT_CONFIRMED_ON_ORANGEPI`，`NOT_PERFORMED` | x86 上的只读路径和保守结论；不能证明 Orange Pi UART6 |

开发机 smoke 观察到的是主机自己的 `/dev/ttyS2`、`/dev/ttyS6` 字符设备元数据，未打开设备。报告中的 `fuser`、设备树和 board identity 不完整，因此总体保持 `unknown`；这不是“候选可用”。摘要见 [20261106_uart_host_smoke.json](20261106_uart_host_smoke.json)。

## 关键验收结论

* 用户现场运行证据：`UXRCE_DDS_CFG=103`，`Starting UXRCE-DDS Client on /dev/ttyS3 @115200`，connected、1 session、43/43 ping、0 send errors。
* 本地源码链：`103 → /dev/ttyS3 → UART4 → PB8/PB9`；`6 → /dev/ttyS5 → USART6 → PC7/PC6`；`104 → /dev/ttyS6 → UART7`。物理接口名 TELEM4 的 USART6 半边不是 PX4 逻辑 TELEM4/104。
* 用户最新线束记录：现用 Orange Pi `/dev/ttyS2` → FC01 RX6/TX6；候选 Orange Pi `/dev/ttyS6` → FC01 RX4/TX4/物理 TELEM3。它与上述运行/源码链冲突，所以状态为 `CONFLICT`，候选为 `UNRESOLVED_DO_NOT_ENABLE`。
* `MAV_0_CONFIG=101`、`/dev/ttyS1` 和独立 USB MAVLink 自动启动不能证明 UART4/UART6 空闲；`MAV_1_CONFIG=0` 也不能否定 USB 实例。
* 当前 uXRCE 为单实例、单选择参数、单 Agent 传输；本轮未实现双串口 DDS、话题分流或带宽叠加。

## 变更范围

本轮修改：

* `uart_preflight.py`：增加分离的用户声明、现场 NSH、Orange Pi 进度、本地 FC01 源码参考和证据冲突汇总；默认保持只读、不开串口、候选不启用。
* `tests/test_uart_preflight.py`：增加 7 项证据冲突/端口编号回归，原有行为测试保持。
* `docs/FC01_DUAL_UART_20261105.md`：重写为 11/06 映射审计，补充完整链路、TELEM4 命名冲突、QGC 只读命令及最小现场证据。
* `README_NAVIGATION_zh.md`、`README_zh.md`、`docs/COMMUNICATIONS_AUDIT_20261103.md`、`docs/TAKEOFF_AUDIT_20261103.md`：移除“最终接线已确认”的操作性表述，指向当前冲突结论。
* `docs/evidence/20261106_uart_mapping_audit.json`、`docs/evidence/20261106_user_nsh_excerpt.txt`、`docs/evidence/20261106_uart_host_smoke.json`：保存来源指纹、用户摘录、审计和主机 smoke。
* `docs/evidence/20261105_uart_source_audit.json`、`docs/evidence/20261105_uart_validation.md`：保留历史内容并增加 superseded/pointer，避免旧记录被误读为当前物理结论。

用户预存的 `native/src/jikl_fc01_system/src/fc01_config_check.cpp` 空白差异未纳入本轮。`git diff --check` 仅报告该预存差异的尾随空格；本轮文件无新增 whitespace 错误。

## 合并与后续

可将本轮提交独立 cherry-pick 到导航修复主线；不需要重启或切换现有导航服务。合并前后都不要修改 `UXRCE_DDS_CFG`、`SER_*_BAUD`、`MAV_*_CONFIG`，不要启动第二 Agent 或刷写 PX4。现场下一步只需补齐：uXRCE 进程 FD/`rc.serial_port`、八针编号和板 revision、必要时现场 `.px4` 构建来源；身份一致后再另行安排候选 UART4 的回环和端到端测试。
