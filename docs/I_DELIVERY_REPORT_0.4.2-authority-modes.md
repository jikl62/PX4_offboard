# Agent I — 0.4.2-authority-modes 交付报告

日期：2026-10-05。结论：**PASS WITH CONDITIONS，仅限离线源码候选交付。真实 Arm / Takeoff / 非零推力 NOT AUTHORIZED。**

实际实现基线为 handoff 中的 `fc01_system_v0.4.1-safety-hardening_source.zip`，不是旧报告推测。handoff 17/17 SHA256 核对通过；00→06 契约、实际 0.4.1 源码、G/H/G2 及旧 F/D 报告已阅读。新候选独立保存；原 0.4.1 和 G2 的全部文件字节及权限仍与输入 ZIP 一致。

本地代码冻结 commit：`f6d544ef955842a7055a2df9ce645ff909fe90cb`。独立审查的初始冻结 head 是 `966a18cf5bd96346970c63e293af49ae0b4bc77d`；基线本地 commit 为 `770e22beb4113e25ffbce3fb00f397517d86b88d`。本地分支仅用于来源记录；未 push/merge 外部仓库，源码包不包含 .git。

## 实际改变

| 部位 | 新行为 |
| --- | --- |
| authority.hpp / Core | 默认 UNSET；显式 RC_TAKEOVER / AUTONOMOUS_NO_RC；记录 mode、attempt identity、lock、takeover；SELF_CHECK 开始锁定 |
| Runtime / console | authority status/rc/autonomous/reset/takeover；模式选择与 reset 只在 live lease、安全地面、输出停止、无未决事务时允许；one-shot 修改在 socket 连接前拒绝 |
| 最终发布边界 | Arm、非零/第二阶段输出在真实 sink 前重新检查模式、锁、attempt identity 和该路线谓词；仍保留四门、health、ownership、Offboard、session、envelope、shutdown 和原 exact-zero 防线 |
| RC adapter | 只读 InputRc / ManualControlSwitches；唯一来源、物理来源、raw timestamp 推进、既有 freshness、loss/failsafe、mapped Kill；不依赖单个 manual_ready |
| 释放与恢复 | mode departure（含 ACK 前）、显式接管、开关位置或计数变化、Kill/termination、failsafe、已建立 Offboard signal loss，停止 target/heartbeat 并单向锁存；RC 飞行丢链也释放；恢复健康不重入 Offboard |
| diagnostics | JSON 显示 mode/attempt/lock、各独立 readiness/confirmation、稳定 reason、takeover；已观察的 Kill 等事实在 step 前就报告阻断；battery/preflight 与 RC reason 区分 |

UNSET 保留 disarmed exact-zero 第一阶段准备；UNSET 永远不能 Arm 或非零推力。第一阶段同样会锁定 UNSET，因此要飞行必须先释放、在安全地面显式 reset，再选择路线并重新准备。成功飞行完成后也需要显式 reset，reset 返回 IDLE/UNSET，绝不启动准备。

RC_TAKEOVER 要求 observed RC link + switch readiness + `rc_takeover_confirmed`。活性需在原 `flags_timeout` 内观察至少两帧，原始信号/sample 时间戳确实推进；首帧、仅 envelope 更新、stale gap 后首帧不放行。遥测不证明校准、映射或物理接管，confirmation 仍必须来自另行验证。已知 Kill 保持，只有更新且有效 mapped OFF raw sample 能清除；NONE/unknown/duplicate/stale 不能清除。

AUTONOMOUS_NO_RC 不要求 RC，却独立要求 `autonomous_failsafe_confirmed`，上游和两个最终边界都检查。选择此模式不确认契约，不写 PX4 参数，不修改 PX4 的原生模式/失效保护。缺失 RC DDS topic 拒绝 RC readiness，不阻塞健康 UNSET stage-one 或已另行确认的自主策略。

普通 Core/adapter/publication 故障的 restart requirement 独立于 takeover latch 且不能清除；先接管再故障也不能通过 reset 恢复。reset 不丢弃 pending 176/400，不能在 Arm/Disarm ACK 未结束时解锁。保留 0.4.1 的健康 session-loss controlled descent exception；接管/RC loss/Kill 优先释放，shutdown 最终边界拒绝输出。未新增自动强制 Disarm 或 companion 手动摇杆。

## 默认值与未改变的保护

VERSION=`0.4.2-authority-modes`，package.xml=`0.4.2`，px4_msgs pin 仍为 `392e831c1f659429ca83902e66820d7094591410`。

交付 YAML 的 `flight_actions_enabled`、`arm_enabled`、`takeoff_enabled`、`flight_parameters_confirmed` 均 false；新增 `rc_takeover_confirmed`、`autonomous_failsafe_confirmed` 均 false。所有既有 YAML 值、21 项飞行数值 -1.0 sentinel、Core 既有 Config/default/validation 原文未变。没有选择真实 hover/gain/thrust/height/speed/tilt/timeout 参数。

`I_INVARIANTS.json` 记录：38 个受保护文件字节和权限一致；`Runtime::ack()`、`track()` 原文一致；controller/reference/wire/publication wrapper/ownership、ground/flight/sim config、socket/lease、install/update/start/deploy 原文一致；G2 的 14 个 frozen headers 与原 0.4.1 一致。install 只构建不可变新 release，绝不自动切 current；本次仅在 fake build/temp-root fixtures 验证这一点。

## 执行验证

| 检查 | 实际结果与范围 |
| --- | --- |
| `bash scripts/test.sh` | 12 个 C++ 程序退出成功；实际 ROS adapter 对类型/API fixture syntax PASS；Python 18 run / 16 PASS / 2 SKIP。C++ 原生 socket bind/listen 子检查同样 SKIP；原生 transport UNKNOWN |
| 新 authority harness | 24/24；直接包含本候选真实 Runtime/Core/最终双 sink；两次运行 JSON byte-identical。正向 Arm/nonzero 仅合成配置及假 sink；无飞控/动力学模型 |
| 原 frozen G2 | 单测 5/5、场景 39/39；仅属于原 0.4.1 baseline，未用旧结果冒充新候选结果 |
| 独立只读 whole-change review | 初始冻结版本 BLOCK；3 Important + 1 Minor 均复现 RED→GREEN 后修正。首帧活性、接管后普通故障、丢中间帧的 switch_changes、即时 JSON/admission 不一致已补回归。未做第二轮独立审查，最终验证为实现方证据 |
| 来源/数值/保护核对 | 17/17 输入 SHA256；原输入字节/权限未变；受保护文件及 ACK/track 相等；false gates / sentinel / pin 检查 PASS |
| patch 与归档完整性 | 对 pristine supplied 0.4.1 应用 `patch --batch --forward -p1`；完整树字节/权限比较；ZIP/TAR 提取等价；无 .git/build/install/log/cache/ELF。实际输出、树摘要和 SHA256 见随包 `PACKAGE_VERIFICATION.txt` |

原始证据在源码 `docs/I_TEST_OUTPUT.txt`、`I_AUTHORITY_HARNESS_RESULTS.json`、`I_HARNESS_DETERMINISM.txt`、`I_G2_LEGACY_TEST_OUTPUT.txt`、`I_G2_BASELINE_RESULTS.json`、`I_REVIEW_REGRESSIONS_RED.txt` / `GREEN.txt`、`I_INVARIANTS.json`。26 项逐条映射见 `I_ACCEPTANCE_RESULTS.md`；审查历史与修正见 `I_CODE_REVIEW.md`；其它开发 RED 证据索引见 `I_RED_GREEN_INDEX.md`。

## 条件与 UNKNOWN

- 新 0.4.2 的真实 Ubuntu22.04/ROS2 Humble build/link 和精确 pin 新消息 schema：UNKNOWN。执行环境是 x86_64、g++ 13.3、Python 3.12，未安装 ros2/colcon；fixture 不能代替真实构建。
- FC01 firmware 是否发布两个新增 RC/switch DDS topic、graph/timing/丢包行为：UNKNOWN。实现保守处理缺失和变化计数，不能证明所有干预都可观测。
- RC 当前未连接；校准、Channel5 Kill、Channel6 Arm、native stick override、实际 pilot takeover/RC loss/Kill 物理效果：UNKNOWN。
- AUTONOMOUS_NO_RC 的 PX4 v1.16 Offboard-loss/RC-loss/estimator/电源失效保护及无 XY 定位情况下的实际动作：UNKNOWN；自主确认默认 false。现场 COM_RC_IN_MODE=4、COM_RC_OVERRIDE=1 不证明 Offboard 接管已可用，官方依据见 `I_OFFICIAL_REFERENCE_CHECK.md`。
- 原生 AF_UNIX listener/client、Orange Pi/aarch64 构建及运行、真实飞行控制数值/动力学：UNKNOWN。便携源码没有 GUI、x86 特定逻辑或硬编码 /home/jikl62；这不构成 Orange Pi PASS。
- 同 UID 的 live INTERACTIVE+PING 保留基线信任模型，不提供恶意同 UID 客户端强认证。历史 socket 的 non-cooperating starter 仍需独立 quiesce，advisory flock 不约束它们。

handoff 的 0.4.1 现场 build 和 disarmed stage-one READY 证据已接受；它们不转为 0.4.2 的真实 PASS。旧 D/F/G/H 历史内容未作为新版本验证结果。

## 六项交付物

1. `fc01_system_v0.4.2-authority-modes_source.zip`
2. `fc01_system_v0.4.2-authority-modes_source.tar.gz`
3. `fc01_system_v0.4.2-authority-modes_vs_0.4.1.patch`
4. `I_DELIVERY_REPORT_0.4.2-authority-modes.md`（与源码内本报告同文）
5. `SHA256SUMS.txt`（列出其它五项的 SHA256，避免自身摘要循环）
6. `PACKAGE_VERIFICATION.txt`

补丁应用到 pristine 0.4.1 解压根目录；新 root 名由归档决定，不由补丁修改目录名。已有脚本文件权限保持基线，应继续使用 `bash scripts/...` 调用。

**未连接 FC01，未启动真实 ROS/Agent 控制链，未 Arm/Takeoff，未发布真实非零推力，未写 PX4 参数，未切真实 current。真实飞行 NOT AUTHORIZED。**
