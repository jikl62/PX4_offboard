# FC01 阶段 1：只读审查与离线复现交接

日期：2026-11-16。工作目录：`/home/jikl62/FC01_OrangePi_Navigation_20261028`。

## 1. 已经确定的结论

当前源码可以在板端已采集的参数下，通过真实 Runtime/Core 的合成 RC 流程进入 **1.5 m HeightHold**，没有发现“源码必然无法完成起飞”的证据。这个结果只验证状态门和控制计算，不是飞行动力学或实机稳定证明。

最新可用香橙派报告确认运行的是 **20261112.1、flight-test、人工 RC 解锁、目标相对高度 1.5 m、视觉关闭**。它的发行清单中 94 个源码文件与当前工作区逐一 SHA256 匹配。用户确认已安装最新版；本轮没有连接板端，不能把 11 月 14 日报告冒充 11 月 16 日实时进程取证。

对这份最新报告，能定位的失败发生在 **RC 解锁后等待导航准入**：journal 的 09:30:06 为 `RC_ARMED_WAIT_CH6_NAVIGATION`，09:30:07 为 `RC_QUEUED_SAFETY_CONDITION`。触发时刻的详细控制器行已经被轮转覆盖，不能确定是哪一项异常。保留的后段详细日志没有 THRUST_RAMP、ASCEND 或 ACTIVE_QUATERNION_RESET。

已经离线复现：

- 达到或经过 1.5 m 不等于完成起飞，必须同时满足高度、速度、离地和连续稳定窗口。
- 活动期姿态重置计数变化，即使四元数数值不变，也会锁存 `ACTIVE_QUATERNION_RESET`，本轮和后续普通健康帧均不继续发布目标和 attitude 心跳。
- 未确认离地、排队不安全事件、旧原始样本不能被健康尾帧或新消息外壳绕过。
- 上述停流可以在 PX4 侧形成 Offboard 失联条件，但 **目前没有同一次最新版异常试飞的完整双端日志，尚不能证明现场就是此因果链**。

## 2. 基线身份与保护范围

| 项目 | 实际核验结果 |
| --- | --- |
| 本地分支 | `readability/fc01-chinese-20261111` |
| 本地 HEAD | `3677b9dd1f35e092e43501d26164c6263545c947` |
| 发行载荷源码提交 | `32990e599a68fde377daadbb89710b77fda1fb82` |
| HEAD 与载荷关系 | HEAD 的后续提交归档发行包和验证材料；94 项载荷源码与现有文件相同 |
| 最新板端报告 | `/home/jikl62/桌面/FC01_navigation_report_20261114_093631_680121.zip` |
| 报告 SHA256 | `ca2a2cc685995169526e08658d552103c4419bccb08c8cb2e7f1a4cf4a7eac4f` |
| 报告中的版本/源码 | `20261112.1` / `32990e599a68fde377daadbb89710b77fda1fb82`，发行清单 dirty=false |
| 板端构建证据 | aarch64、ROS Humble、2026-11-14 07:32:32 UTC；输入摘要 `ca7773e4bf68d8ece22a8194d6d7c047a66aa372cb659e7e03201eee657cca1e` |
| 部署证据中的控制器 SHA256 | `b3568e9afb8e37c50fb1601d563d9d6316633deda726b9cc792f78124cc60bbe` |
| 板端 SDK | `/home/orangepi/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE` |

服务进程路径、版本状态、构建清单互相一致，但报告没有运行中 `/proc/2005/exe` 的独立哈希，也没有板端 Git 分支取证。上面的二进制哈希来自部署证据，不能升级成当时运行内存映像的哈希。`FC01_RECOVERY_DIAGNOSTIC_V5_20261110` 是能力标识，不能据字符串里的日期认定运行了旧源码。

开始时已有未跟踪的 `tests/test_micovio_clock_candidate.py`、`tests/test_micovio_uart0_reader.py` 和 `tools/`，本轮未改动、未纳入提交。本轮只新增本调查目录中的报告、解析脚本和离线证据；生产源码和配置的 tracked diff 保持为空，HEAD 不变。没有提交、推送、部署、启动控制服务、SSH、串口设置、解锁或动力操作。

复核索引：[BASELINE.json](evidence/BASELINE.json)、[COMPANION_RECHECK.json](evidence/COMPANION_RECHECK.json)。早期粗索引 [COMPANION_ARCHIVES.json](evidence/COMPANION_ARCHIVES.json) 还保留全部 phase_event；其中合计行数不能理解成独立状态样本数，以 RECHECK 分列的快照和事件数为准。

## 3. 实际服务入口、配置和触发方式

报告中 `fc01-navigation.service` 主进程为 PID 766：

```text
/usr/bin/python3 /home/orangepi/FC01_OrangePi_Navigation_20261112_release/navigation_service.py
  run --mode flight-test --confirm-flight-test

Agent PID 2002:
/home/orangepi/fc01_agent_build_offline/MicroXRCEAgent
  serial -D /dev/ttyS2 -b 115200 -v 0

控制器 PID 2005:
/home/orangepi/FC01_OrangePi_Navigation_20261112_release/install/lib/jikl_fc01_system/fc01_system_node
  --ros-args --disable-external-lib-logs
  --params-file /home/orangepi/FC01_navigation/runtime.yaml
  -p socket_path:=/home/orangepi/.cache/fc01_operator/ipc/s_pow_pwqb/control.sock
```

生成代码的调用链：

1. `navigation_install.py:31 render_unit`，第 56 行生成 `navigation_boot.sh run --mode …`。报告有 systemctl status，未附原始 unit 全文，不能只凭生成器断言板端 unit 从未手动改写。
2. `navigation_boot.sh:4–9` 加载 `navigation_environment.sh` 后 exec 服务 Python。
3. `navigation_service.py:36 candidate_config_path` 选择 `native/config/flight_test.yaml`；`load_profile:161` 读取 `navigation_settings.json` 的目标高度/导航档覆盖，经 `navigation_profile:91` 合成运行权限。
4. `navigation_service.py:319` 使用 `/home/orangepi/FC01_navigation/runtime.yaml`。因此要核对实际 runtime，不能只看当前编辑器打开的 ground/attitude YAML。
5. `controller_path:140` 指向本版 `entry/install`，服务由 standby/Supervisor 管理控制器和既有 Agent。状态窗口不拥有起飞授权。

板端 runtime 的有效值见 [runtime_20261114.yaml](evidence/runtime_20261114.yaml)：

| 配置 | 有效值 |
| --- | --- |
| profile / 模式 | `attitude_altitude` / `flight-test` |
| 动作/起飞门 | `flight_actions_enabled=true`、`takeoff_enabled=true` |
| 程序解锁/通用路线/视觉 | `arm_enabled=false`、`flight_enabled=false`、`visual_navigation_enabled=false` |
| 人工 RC 路线 | enabled=true、takeover confirmed=true、autonomous failsafe confirmed=false |
| 目标 | `rc_target_height=1.5` m，相对本次地面锚点向上 |
| 导航选择 | `rc_navigation_mode_slot=1`；是 PX4 六档选择位置，**不是 CH6 通道号** |
| 推力 | hover=.50、reserve=.04、min=.12、max=.80、slew=.40/s |
| 垂向增益 | height=.50、velocity=.15、integral=.03、integral_limit=.08 |
| 轨迹速度 | climb=.30 m/s、descent=.20 m/s |
| 时间 | 20 Hz、stall=.30 s、ground=2 s、prestream=1.2 s、armed dwell=1 s |
| 稳定/期限 | 高度 ±.15 m、速度 ±.15 m/s、连续 2 s；Ascend 10 s、任务 30 s |

实际启动顺序由 `rc_navigation.hpp:208 RcNavigationStart::step` 决定：先未解锁 Stabilized、安全开关、静止和新地面证据，再完成零推力预发送；看到 READY 后需要新的人工解锁开关边沿，并由 PX4 新 status、`armed_time` 和 arming reason=2 证明人工 RC 解锁；随后新的导航选择和真实 Offboard 状态配对，才进入 ArmedGround。人工解锁到导航窗口 8 s。

CH5/CH6 是现有提示文本；真正使用的事实是 PX4 `manual_control_switches.arm_switch/mode_slot` 和 manual source，不应把提示文字当作本轮重新核验过的物理接收机映射。`Runtime::request:720` 对此 RC 路线拒绝控制台 TAKEOFF/程序模式接管，不能从 UI 绕开。离线正常序列也断言自动 Arm/Offboard 命令均为 0。

## 4. EKF2 → DDS → 控制器的高度链

PX4 语义以 `/home/jikl62/fc01_px4_dds_build/PX4-DM-FC01-v1.16` 的 `07983bfb894b9372d7520d43ac10dc792d0b4b9f` 为依据，匹配 12 份直接路径硬件 ULog。另一份 `/home/jikl62/PX4-Autopilot` 当前是较新开发分支，不能混用。匹配目录仍有 DDS topic/ping 两处未提交改动，不能推断它们已刷入；完整说明在 [PX4_CHAIN_zh.md](PX4_CHAIN_zh.md)。

| 文件/函数 | 跟踪结果与维护注意点 |
| --- | --- |
| PX4 `EKF2.cpp:1538 PublishLocalPosition` | z 为融合后 NED 位置，m、向下正；vz 为融合后 NED 速度，m/s、向下正；z_deriv 是位置导数，不能假定与 vz 数值相同 |
| 同文件 1570–1571 | z_valid/v_z_valid 均采用垂向位置有效 **或** 垂向速度有效的兼容策略；两个 true 不代表两路独立传感器都可靠 |
| 同文件 1542、1657 | timestamp_sample 是本轮 EKF 更新传入的采样标记，timestamp 是发布 hrt；它不是 MTF 测距仪的原始采样戳 |
| 同文件 1603、1624 | dist_bottom 另有有效性；发布 z/vz reset counter 和 delta，不等于任务地面高度 |
| uXRCE `dds_topics.h.em` / `Tools/msg/templates/ucdr/msg.h.em` | 发出 timestamp 和 timestamp_sample 时都加 Agent time offset；回传心跳减 offset；ULog 启动时钟、同步后 ROS 微秒、主机 monotonic 秒不能直接相减 |
| `fc01_system_node.cpp:216` 本地位置订阅 | 接收 `/fmu/out/vehicle_local_position`，保留 envelope 与 timestamp_sample；status 用 `/fmu/out/vehicle_status_v1` |
| 同文件 `receive_sampled:936`、`populate_attitude_feedback:1442` | 分别判断源年龄、原始样本进展、接收年龄、源戳回退；把 local.z/vz/z_deriv/reset/valid 原样传入 |
| `source_clock.hpp:37–111` | 新外壳不能给重复 raw sample 续鲜；原始戳回退锁存；同步域未来容差 100 ms，并非额外 TTL |
| `Core::health:2426` | 源时钟、唯一来源、status/land/flags、PX4 failsafe、姿态、local/z/vz valid 和有限值全部检查；只检查一个 z 数值不足以授权 |
| `SystemNode::update_stationary_evidence:1333` | 未解锁、landed、at_rest、新鲜 local 和 abs(vz)≤.10 m/s 累计；垂向 reset 清地面计时 |
| `RcNavigationStart::observe_ground_reference:434` | 至少 3 local、3 land、2 status、跨 2 s；检查高度窗口、独立源进展和位置导数一致性。未解锁 READY 中可更新锚点；解锁意图后不准重建 |
| `Core::rc_startup:2717` | 继承已证明的 RC 锚点 z0 和姿态/reset，Joined 后进入 ArmedGround，目标来自 rc_target_height |
| `Core::step:1442` / `vertical_controller.hpp:step` | 相对高度 `height=z0-z`，向上速度 `speed=-vz`；FRD 推力写成 `{0,0,-幅值}`。1.5 m 是相对任务地面，不是要求 local.z=-1.5 |
| `Core::advance_thrust_ramp:2092` | 先处理超时，再按 2 s 到 hover、后 2 s 由 hover 增至 hover+reserve 的计划；受 slew/包络约束。高度达到 .12 m 可提前接闭环制动，不能据此正式认证离地 |
| `takeoff_evidence.hpp:observe` | 新 !landed 检测至少两帧，源/主机都满 .2 s；新 local 要高度≥.12 m、向上速度≥.08 m/s、持续 .2 s 且确有高度增长；缓存消息不算证据 |
| `Core::step:1689` | Ascend 中轨迹已到目标、liftoff confirmed、!landed、实测高度 ±.15 m、速度 ±.15 m/s **连续 2 s** 才进入 HeightHold |
| `Core::step:1607` | Ascend 进入后 10 s 未完成，沿原受监督下降；不是无限等待，也不是立即切零动力 |
| `vertical_feedback_guard.hpp:observe` | vz 与 z_deriv 速度差仅诊断；位置变化与 z_deriv 积分不一致等硬失败仍可停流。不能把旧 VERTICAL_VELOCITY_INCONSISTENT 名称套给当前全部路径 |

关键区别：MTF 已直接接入 PX4，FC01 此路线用的是 **PX4 融合后的 local z/vz**。FC01 没有直接选 MTF 原始 ToF 作高度。Position 实飞正常是用户当前硬件基线，但不能单独证明 DDS 时钟/采样、人工 RC 准入和外部姿态发布路径也正常。

## 5. 按记录分别审查，禁止跨次拼接

### A. 最新源码板端报告：20261112.1

systemctl/journal：09:17:15 启动模式/高度/权限明确；09:17:16 Agent ttyS2@115200 connected；09:17:31 READY。09:29:22 提前解锁/导航触发 `RC_ARM_OR_NAVIGATION_BEFORE_READY`；09:30:06 等待导航，09:30:07 `RC_QUEUED_SAFETY_CONDITION`；09:30:16 又 READY。09:30:27 还有 `RC_PRESTREAM_GROUND_CHANGED`。

详细文件保留约 09:31:52–09:36:31：`.2` 为 233 个快照 +233 个 phase_event；`.1` 为 232 个快照 +241 个 phase_event；当前日志为 92 个快照 +91 个 phase_event。事件有 before=after 的周期行，不能把它们都当状态切换。详细快照仅包含 READY、IDLE、PRESTREAM；最终状态 READY_WAIT_ARM、feedback_current=true、controller PID=2005。不能据后段健康证明 09:30 的瞬时坏事件不存在。

`RC_QUEUED_SAFETY_CONDITION` 来自 `attitude_altitude_core.hpp:1118–1122` 的多项合并：Kill、failsafe、calibration、HIL、preflight、电池、RC 来源/新鲜度、轴/摇杆、辅助开关。现有文本无法区分是哪一项；缺少触发时原始事件，不能定为串口故障、地面零点故障或用户拨错开关。

### B. 旧源码板端报告：20261110.4，单独保留参考

文件 `FC01_navigation_report_20261021_152318_747738.zip`，SHA256 `3559497dff230940f50a11b60b2e273729f2196252e8ec19ef34f9d1e67312c5`；来源提交 `aecb079f25a63983c6f504bd0a685da7f70fe707`、dirty=true，与当前源码不等价。

其同一控制器 PID 2431、同一 monotonic 序列的 phase_event：

| 单调秒 | 阶段变化/观察 |
| ---: | --- |
| 153.678852 | READY → RC_ARMED_WAIT_NAVIGATION |
| 155.233497 | → ArmedGround |
| 156.387289 | → ThrustRamp |
| 159.047454 | → Ascend，离地已确认 |
| 169.092418 | → Descent，`ASCENT_TIMEOUT_SUPERVISED_RECOVERY`，当轮仍有本地目标和心跳提交 |
| 170.298412 | → Safety，`OPERATOR_TAKEOVER_LATCHED`，当轮无提交；最后本地心跳提交 170.249501 s |

z0=.557554543 m；20 个 Ascend 快照显示相对高度 .2698–2.0621 m，全部 target_height_settled=false。超时前最后一个快照高 1.82798 m、NED vz=+.10961 m/s，已超过目标的 .15 m 高度容差。能够证明“这一旧任务进入上升后未完成稳定条件并超时下降”，不能从稀疏快照证明每个控制周期的唯一阻塞项，更不能用它解释当前最新版。未配得对应 ULog，不对齐 PX4 心跳或 No offboard signal。

### C. 硬件 ULog 与扩大检索

直接位于主目录/桌面的 12 份 ULog 全部解析成功，固件哈希为 `07983bfb…`，没有 Position(nav_state=2)，6 份有历史 Offboard。完整逐份模式、reset、心跳与事件见 [ULog 审查](evidence/ulog/README.md)；同次 PX4 事件排序见 [PX4_TIMELINES.json](evidence/PX4_TIMELINES.json)。这些序列没有匹配的当前 companion 试飞身份，不做双端对齐。

最新文件 `log_10_2026-11-15-16-47-56.ulg`：SHA256 `8d510d25fbea1e6473939765532ca051705f18cbf967a740fbe05f992490ee11`，7.677352 s。17 个 status 均 Stabilized(nav_state=15)，无 OffboardControlMode 记录；boot65.835730 s ping exhausted、66.836129 s DDS session setup failed、66.836591 s ttyS3@115200 重新初始化。它证明该片段有 DDS 建连失败，不能直接断言 UART 物理静默或当前连接仍失败。

76 个 local 样本 z 从 +.096944585 到 -1.485310912 m；vz 最低 -.51962 m/s；z_valid/v_z_valid 全 true、z/vz reset=1/1、姿态 reset=2 恒定。RC 上锁在 boot69.672397 s，后面还有 10 个 local 样本，不能把全部高度变化当作解锁飞行。dist_bottom_valid 全 false；仅凭 z 变化不能区分真实上升与估计漂移。该日志不能验证 ACTIVE_QUATERNION_RESET 的现场因果。

扩大 `rg --files --hidden` 检索共 361 个 .ulg 路径，排除回收站/PX4 示例后 61 路径、30 个唯一内容均完成模式分类。唯一额外 Position 记录 `FC01_M1_S10_failure.ulg` 经内容核验是 **PX4_SITL**，不同固件 `8714f244…`；其约 5 s Position 处于 failsafe/降落，不能当作真实稳定 Position 基准。见 [补充分析](evidence/ulog_extra/README.md) 和 [库存](evidence/ULOG_INVENTORY.json)。后续通过保存的 inventory_ulogs.py 重复分类，30 个哈希/模式/Position 区间一致；[复查库存](evidence/ULOG_INVENTORY_RECHECK.json) 补存准确检索命令、排除路径和检索 stderr。桌面 4 个 ZIP 的目录索引没有 .ulg 成员，见 [ZIP 索引](evidence/DESKTOP_ZIP_ULOG_INDEX.json)。未解开其他位置全部压缩归档、未读取外接 U 盘和飞控 SD 卡，不声称机器或用户手上不存在其他日志。

## 6. ACTIVE_QUATERNION_RESET 与 Offboard 丢失的可验证关系

`AttitudeReference::observe:37–43` 在参考已建立、活动期 reset counter 改变时拒绝，不要求实际姿态发生大角度变化，也不使用 delta_q_reset 做连续补偿。`Core::step:1283–1287` 写故障后返回空计划；排队姿态事件还可在 `observe_safety_event:1068` 更早锁存。`Safety` 后续 step 只收集恢复地面证据，不靠普通好帧清故障。

`Runtime::step:1084` 接受空计划；`SystemNode::tick:474` 每轮先清旧提交输出，仅 `output.attitude.publish` 为真才在 503 行进入目标/心跳块。正常块是目标先、心跳后、分别取最终上下文，非原子；部分失败走 `PARTIAL_OR_FAILED_SETPOINT_PUBLICATION`。本次复现的假 sink 使用固定上下文，没有模拟 ROS 回调或网络；相关现有测试另外验证 522 种发布组合。

匹配 PX4 源码 `offboardCheck.cpp:46` **直接按 COM_OF_LOSS_T 判断心跳年龄**；不能说“先固定 500 ms 再等 COM_OF_LOSS_T”。modeCheck 在需要 Offboard 的模式条件失败时报 `No offboard signal`，failsafe 再按同次 COM_OBL_RC_ACT 和其他健康条件决定行为。源码中的另一处 500 ms 是 flags 定期发布条件。最新短硬件 ULog 的 COM_OF_LOSS_T=1.0，但它没有 Offboard，不能据此测当前停流的延迟。

因此，`ACTIVE_QUATERNION_RESET → companion 停流 → PX4 超时 → No offboard signal` 是源码支持的候选链。另一方向“先 DDS/调度停流或先模式/控制权变化，再发生故障”也不能在缺同期日志时排除。`No offboard signal` 本身不是串口损坏的特异诊断；本地 publish 返回也不是 PX4 实际接收证明。

## 7. 复现方法与实际结果

所有命令在开发机执行，不链接 ROS，不连接 PX4。工作目录为本项目根目录。

```bash
python3 docs/investigations/20261116_stage1/repro/inspect_companion.py

g++ -std=c++17 -O2 -Wall -Wextra \
  -I native/src/jikl_fc01_system/include -I native/tests \
  docs/investigations/20261116_stage1/repro/reproduce.cpp \
  -o /tmp/fc01-stage1-reproduce
/tmp/fc01-stage1-reproduce > /tmp/fc01-stage1-reproduction.jsonl

python3 docs/investigations/20261116_stage1/repro/run_targeted_tests.py \
  --output /tmp/fc01-stage1-targeted-recheck

python3 docs/investigations/20261116_stage1/evidence/ulog/inspect_ulogs.py \
  --output /tmp/fc01-stage1-ulog-recheck
python3 docs/investigations/20261116_stage1/evidence/ulog_extra/inspect_extra.py
python3 docs/investigations/20261116_stage1/repro/inventory_ulogs.py \
  --output /tmp/fc01-stage1-inventory-recheck

g++ -std=c++17 -O2 -I native/src/jikl_fc01_system/include \
  native/tests/vertical_feedback_replay.cpp -o /tmp/fc01-stage1-vertical-feedback-replay
python3 docs/investigations/20261116_stage1/repro/replay_logged_vertical.py
```

首次复现参数从板端 runtime 和生产 `attitude_ros_parameters.hpp` 映射生成，64 项姿态参数一致；未设置字段沿生产默认值。它绕过 ROS 参数加载和源时间适配，把反馈有效位作为合成事实提供，不是完整节点仿真。初版五组经独立代理复跑、447 行完全一致；增加两组排队事件后由主代理重复执行，随后最终独立审查也重新编译执行，**449 行逐字节一致**。两个版本的审查范围分别保留在独立记录中。

| 用例 | 真实观察 |
| --- | --- |
| 正常 1.5 m | t3.95 ArmedGround、5.00 Ramp、5.85 Ascend、14.55 HeightHold；自动 Arm/Offboard=0 |
| 相同 q、reset+1 | t14.60 Safety/ACTIVE_QUATERNION_RESET；目标/心跳计数均停在 291；20 个后续好帧无新增提交，z0 不变 |
| landed 保持 true | .13–.24 m 的新高度和 .20 m/s 向上速度在 .6 s 观察内仍是 Ramp、离地未确认，继续控制发布；不代表可永久等待，另有超时恢复 |
| 经过目标未稳定 | 上升经过 1.5 m 后停在 1.70 m，超出容差；t15.85 Descent/ASCENT_TIMEOUT_SUPERVISED_RECOVERY，当轮仍发布 |
| 排队 failsafe 后健康 | 等待导航中锁存 RC_QUEUED_SAFETY_CONDITION，健康尾帧无输出 |
| 排队 moving sticks 后健康 | 得到相同故障原因，证明不能靠这个通用原因反推唯一输入 |
| 原始样本/回退 | 新外壳不推进旧 raw 的 received；源回退后新好样本不能清 regressed |

结果：[reproduction.jsonl](evidence/reproduction.jsonl)、[stderr](evidence/reproduction.stderr)、[重复核验](evidence/REPRODUCTION_CHECK.json)、[初版独立复核](evidence/INDEPENDENT_REVIEW_initial_zh.md)、[最终独立复核](evidence/INDEPENDENT_REVIEW_final_zh.md)。最终审查指出的一处表格分隔符和推力措辞已在本报告修正。

8 个已有 C++ 测试程序全部编译/运行退出 0：takeoff_evidence、r632_ramp_height_guard、r632_height_tracking、vertical_feedback_guard、rc_navigation、rc_takeoff_stage、r62c_review_clock、stage3_publication_combinations。最后一个报告 **522 组合通过**，反馈守卫报告 7 组，时钟报告 10 个通过项；不把这些不同粒度相加成总“测试数量”。编译命令和完整输出见 [targeted_tests/results.json](evidence/targeted_tests/results.json)。

将 12 份 ULog 导出的 timestamp_sample/z/vz/z_deriv 按垂向 reset 分段，重放当前生产 guard：13 段中 5 段保持健康，8 段触发 VERTICAL_POSITION_RATE_INCONSISTENT；最新 76 样本段健康。结果全保存在 [vertical_replay/results.json](evidence/vertical_replay/results.json)。这是对已记录、可能降采样的估计器输出进行一致性检验，不是每一真实 DDS 回调重放，也未按当时控制阶段过滤，因此不能称为已复现这 8 次实飞保护误报，不能据此降低阈值。

本次复现另用 `-O1 -g -fsanitize=undefined -fno-sanitize-recover=all` 编译/运行，退出 0、无 UBSan 诊断，449 行输出与普通编译逐字节相同，见 [SANITIZER_CHECK.json](evidence/SANITIZER_CHECK.json)。这不能代替 ASan、TSan、ROS 调度或硬件验证。本阶段没有执行 ASan/TSan、全量 ROS 构建、硬件通信或新实飞，也没有把旧发行验证当成本轮重新执行。

## 8. 阶段 2 的接续清单

| 优先级/性质 | 已知内容 | 下一步可执行工作 |
| --- | --- | --- |
| P1 审查事项，非已确认缺陷 | 活动期参考重置直接停流；PX4 的最终退出行为依赖有效心跳与失败动作参数 | 先取得同次 reset、delta_q、心跳、mode/failsafe 序列，评估既有保护与交接合同；不能直接删 reset 门或静默续发旧参考 |
| P2 可观测性问题 | 最新报告的通用 queued 原因无法区分输入，关键事件被轮转覆盖 | 离线扩展输入组合；若阶段 2 要改诊断，独立保留触发源/失败位/原始 source stamp 和 boot/PID，避免增加控制循环同步 I/O |
| P2 待核验证据 | 部分旧日志的 z 与 z_deriv 积分不符；仅 validity=true 不足以排除估计器变化 | 用真正稳定 Position 的同配置 ULog，比较 armed/airborne 段和全速 DDS 样本，不改用户已稳定的参数 |
| 已有安全保护 | 单帧高度/landed 缓存不能证明离地，需连续稳定才到高；超时进入下降 | 保留保护；对真正失败的那次逐条列出未满足的谓词和源年龄 |
| P3 证据身份改进 | 报告缺运行中 executable 哈希和完整 unit | 后续无动力取证补实际 unit、PID/exe 哈希、runtime/hash、发行清单和 boot_id，避免仅看目录名/能力标识 |

本轮未确认新的 P0 代码缺陷；也没有证据支持把参考重置一概认作无害。尚缺资料应作为阶段 2 输入缺口保留，**无需等待回复才继续可独立完成的源码和合成测试**：

1. 用户所说的当前稳定 Position 实飞 ULog，包含同次参数、机体/固件身份、MTF 距离/质量及实际融合状态。旧 SITL 不可替代。
2. 最新版异常任务从启动、READY、人工解锁、导航、升力阶段直到退出的完整 ULog + `controller.log*` + journal，配同一 boot/PID/源码身份；尤其 09:30 触发的详细事件，现有归档已丢失，无法补造。
3. 同次姿态 reset counter / delta_q_reset、z/vz reset、最后有效 OffboardControlMode、flags/event 和同次 COM_OF_LOSS_T/COM_OBL_RC_ACT。日志末样本不自动等于最后网络包。
4. PX4 boot µs 与 ROS 同步后时间/主机 monotonic 的可靠对应点；不能靠文件名或两台机器墙钟直接对齐。
5. `FC01_navigation_report_20261114_074210_900794.zip` 在历史终端文字中被提及，但本轮文件清单没有该归档；文字里的退出/恢复状态不能替代完整报告。

阶段 2 应从本报告定位到的实际函数和可复现输入开始，先区分 **未达到准入、离地待证、到高未稳定、主动安全停流、DDS 未收到** 五类情况。继续保留原有稳定 Position 参数、串口和人工解锁检查结果，不重复调参、换高度源或启动动力验证。本阶段到交接为止，不自动实施飞行行为修复。
