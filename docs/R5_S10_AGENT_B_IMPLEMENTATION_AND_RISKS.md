# FC01 M1 S10 · Agent B 增量 DEV 实施报告

日期：2026-10-09。版本：`0.4.4-offboard-flight-dev-r5-s10-dev`。
**仅独立离线 DEV，不是飞行批准；未连接真实 FC01、未运行 Arm、未更改真实参数。**

## 1. 来源与隔离

- 唯一实施基线：DATA 压缩包内 `S10_MODIFIED_SOURCE/`；`VERSION` 原值为 `0.4.4-offboard-flight-dev-r4`，但头文件已存在 `land_sample_max_gap`，属 S10 实测 1 Hz 修补版，不等于原始 R4。
- 原始 R4：STARTER 中 `R4_BASELINE_REFERENCE/R4_SOURCE_original.zip`，仅用于对照，与 DATA 自带 `BASELINE_R4` 压缩包 SHA256 完全相同。R4 自带 `workspace/src/px4_msgs/.git`，离线 `git rev-parse HEAD` 确认 `392e831c1f659429ca83902e66820d7094591410`；**S10 采集包没有 `.git` 元数据，但其所有仍存在的 `px4_msgs` 工作树文件与 R4 逐字节一致**。源代码补丁有意跳过 R4 独有的 `.git` 历史/二进制元数据，完整源码 ZIP 以 S10 为基线。
- 本 DEV 为新目录，未就地覆盖 S10 或 R4。源码树保留全部既有测试、配置、`workspace/src/px4_msgs`、Node、Runtime、PublicationGuard。
- 版本命名遵守既有 `test_tools.py`/install.sh 的 `0.4.4-offboard-flight-dev-` 前缀，最终版本是 `...-r5-s10-dev`，非上位系统正式 release。

## 2. S10 源码路径审计

### THRUST_RAMP → ASCEND 全部实质条件

见 `attitude_altitude_core.hpp` 的 `Core::step` / `dwell_land`：

1. 所有通用门槛通过：20 Hz 控制步距未失速、健康与 fresh 检查、无 PX4 failsafe/time regression/冲突、无失联 takeover、四个 flight gates 继续打开、battery_ready、正确 authority、有效水平姿态参考、Z/VZ 与高度/倾角在限定范围；
2. 仍在 `ThrustRamp` 且未先触发 `ascent_timeout`；`vertical_->ramp()` 返回健康，未持久饱和；
3. `s.landed == false`；`land_stamp > land_baseline_`（ArmedGround 进入 ramp 时的检测器基线）；
4. `z0 - s.z >= liftoff_height_m`；`-s.vz >= liftoff_speed_mps`；
5. `dwell_land()` 至少接收到两个严格递增的 landed-detector 时间戳，两个当前样本均满足联合条件，消息实际到达间隔不超过 `min(land_sample_max_gap, land_timeout/2)`，累计驻留达到 `liftoff_dwell_seconds`。

原 S10 SITL 覆盖 1 Hz 的配置 `land_sample_max_gap=1.25`, `land_timeout=2.5`, `liftoff_dwell_seconds=0.20`。**0.20 秒是最小累计时间，不代表 1 Hz 会在 0.20 秒内确认。**在 1 Hz 连续采样下，理想两次有效 `land_stamp` 可用，通常至少横跨约 1 秒。PX4 `Takeoff detected` 与 M1 的升空条件不是同一个判据，因此不能由 PX4 该日志单独断定 R4 已进入 ASCEND。

### 原 S10 的独立 bug：反弹后复用同一检测器样本

在 20 Hz Core 连续运行时，原 `dwell_land(false,...)` 立即清空 dwell，但**没有记录触发否定的 detector stamp**。若 landed/height/speed 缓存条件在两帧 detector 更新之间再次变为真，原函数可能把这帧旧 stamp 当成第一帧新肯定证据，后一新 detector stamp 即完成 dwell。新方案加入 `land_dwell_blocked_stamp_` 阻止**已出现否定条件的同一 land stamp**重入积极证据；与 `land_baseline_` 联合，要求真实新消息。并且仅在**新的 stamp**到来时处理超长间隔并重建基线，不再用循环中重复的旧消息重置后重新取证。精确 1 Hz 时 `double` 时钟的 nanosecond 容差 `1e-9` 只消除浮点迭代误差，不免除两帧约束。

### 起飞与下降超时

- `ThrustRamp` 超过 `ascent_timeout`：仍保持 S10 已有处理，依据 `landed/height/speed` 进入 `PoweredGround`（下降推力）或 `Descent`（闭环控制），同周期保持 slew 连续。**真实 S10 因为何种数据未进入 ASCEND 不能由源码判定，须 A 的 ULog。**
- **原 R4/S10 `Ascend` 超时 bug**：原先 `fault("ASCENT_TIMEOUT")` 导致同周期空输出，未先受控退出。R5 在 telemetry/authority 仍健康时，通过既有 `descend(s,now)` 进入 `PoweredGround` 或 `Descent` 并立即输出受限的推力/心跳，无新 flight mode/400/176 命令。
- `Descent` 超过 `descent_timeout`：当前 DEV **仍停止 Offboard 发布**，但显式归类 `UnknownAirborne`，reason 为 `DESCENT_TIMEOUT_ARMED_OFFBOARD_PX4_HANDOFF_UNVERIFIED`。这不是确认 PX4 正式接管，只是有限控制合同失败后的 PX4 失联保护依赖。该路径保留为 **P0 BLOCKER**，未经真实 SITL 独立模式/故障测试不得放飞。**没有**简单无限续心跳、删除 fail-safe，也没有在空中发送未经验证的 176/21/400。

### 源码到“无 Offboard 信号”的责任链

`Core::step(Descent)` 达到超时 → `Core::fault()` 改 `phase=Safety` → 当前 `Output{}`（`publish=false`）→ `Runtime::step()` 返回空 `out.attitude` → Node 的 `tick()` 不再调用 `publish_attitude_at_boundary` → `/fmu/in/offboard_control_mode` 心跳停止 → PX4 根据 `COM_OF_LOSS_T` 与 `COM_OBL_RC_ACT` 自主触发 Offboard-loss 处理。这里**代码导致心跳停止可确认**，PX4 最终选中 Position、Land 或其他动作由其实际可用估计、参数和 failsafe 状态决定，不可仅由 Core 证明。

### Ground/Air 新鲜度边界

- `Stamp::accept` 严格递增，重复/回退帧不刷新 `received`；回退锁 `regressed=true`。
- Node 在 `snapshot(now)` 使用真实到达时间评价 `land_fresh`，并与 `status_fresh/local_fresh/failsafe_fresh` 形成 Core health gate。
- dwell 不是订阅器本身新鲜度：dwell 为跨帧证明，而 `fresh()` 是对最后收到的新时间戳年龄的上限。它们不可互相替代。
- 原有 `ArmedGround` 容许 Arm 后 `at_rest=false`，仍要求 `armed/landed/offboard`、高度速度稳定与多帧驻留。检测器 stamp 冻结不应进入 ThrustRamp。

## 3. 变动文件（与 S10 比较）

- `VERSION`：DEV 后缀；保持历史命名模式。
- `config/S10_1HZ_EXPERIMENTAL_SIM_ONLY.yaml`：隔离保存来自数据包的真实 S10 SITL 参数快照，顶部明确仅模拟，四个正式 flight gates 依旧 default false；不是动力校准依据。
- `src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp`：仅变更离地/着陆驻留证据处理和 `Ascend/Descent` 超时行为；未动 RC、ACK、DDS、400 TransactionTracker、最终 sink gate。
- `tests/test_m1_s10_1hz_recovery.cpp`：合成正/反例和肯定式断言（包括恢复中连续发布、超时后明确没有“已接管”保证）。
- `tools/s10_timeline.py`：只读解析 Node JSONL/CSV 控制日志，**不**解析 ULog、不连接设备。
- `tests/test_s10_timeline.py`：离线解析器 fixture。
- `docs/`：本说明与 C 独立审查任务书。

S10 原有的 C01–C12 原始反例及测试脚本未修改。与 R4 比较还含原 S10 的 1 Hz 已实施补丁；不能把原 S10 修补归因于本轮 R5。

## 4. 验证边界与风险矩阵

| 验证项目 | 结果 | 证据/限制 |
|---|---|---|
| R5 离线 C++ 严格编译与执行 | PASS：18/18 可执行文件 | `R5_CPP_REGRESSIONS.log`；`g++ -std=c++17 -Wall -Wextra -Werror -pedantic` |
| R5 新增 1 Hz 正/负例 | PASS：6 个报告段 | `R5_S10_POSITIVE_AND_TIMEOUT.log`；含反弹、冻结、回退、两个证据样本、受限恢复与超时 |
| C11/C12 原反例重放 | PASS：旧漏洞断言失败（预期） | `R5_C11_C12_REPLAY.log`；**不要**把 abort 当作新的回归失败，正向安全依据是 `test_m1_r4_ack_isolation` |
| ROS adapter 编译 | PASS：fixture syntax only | `R5_ADAPTER_FIXTURE.log`，并非实际 Humble 构建 |
| Python tests | PASS：全部 19/19（含新测试） | `R5_PY_ALL.log` |
| ROS 2 Humble `colcon build` / DDS ABI | UNKNOWN / 未执行 | 本执行环境无 `/opt/ros/humble` 和 `colcon`，不可把原 DATA 的 `BUILD.log` 当本候选构建 |
| PX4 X500 SITL 实际飞行回归 | NOT RUN | 本轮限制在纯离线，不控制飞行 |
| 动力、hover_thrust、battery warning 因果 | UNKNOWN / A 数据待交接 | 不修改任何实测控制参数 |
| 真实 FC01 手动/自动 failsafe / mode handoff | BLOCKER | 无验证的空中交接方案，STOP / power loss / descending timeout 不能宣称安全 |

## 5. 不得掩盖的阻断风险

1. **P0：DESCENT_TIMEOUT 仍导致 armed Offboard 心跳停止，实际 PX4 接管行为与安全性未验证。** 这不是已修复安全性；只是根因明确、诊断明确。必须由独立仿真评估降落模式可用性和 Offboard-loss 参数，并由 C 审查是否允许这种有限故障契约。不能靠无限心跳或盲目 Disarm 掩盖。
2. **P0：S10 进入 ASCEND 失败的真实物理原因未知。** 需要智能体 A 的 ULog 时间对齐结论、有效电池数据、实际高度速度与 actuator/thrust 曲线才可改变增益/推力。
3. **P0：完整 Humble native build 未完成。** fixture 不能证明 ROS2 topic/API 编译链接。
4. **P1：Land detector 1 Hz 与 Core 20 Hz 在丢帧、多源时间戳、重放/跳变、多估计器重置时的实机行为待实证。** 当前只证明合成时序。
5. **P1：高度估计、battery warning、空气动力、ESC 失配、真实臂姿态、横向漂移与 PX4 fallback Position 能否成立**全属 UNKNOWN。
6. **P1：ROS adapter 审查仍需 C 检验 176、400、21、Authority、两个发布 sink 的实际 final boundary 保护，尤其瞬态失联与控制权仲裁。** 本轮未改这些逻辑。

## 6. A→B 进一步修复准入条件

需要 A 的 `A_TO_B_FIX_CONTRACT.md` 标明：每条结论的数据文件 SHA/ULog 内 message/topic、事件时间戳与同步方法、量纲、置信度、可用/缺失数据、是否能区分推力不足、离地未确认、估计异常、供电告警、对应允许改动范围。未收到前不修改 `aa_hover_thrust`, thrust min/max, hover gains, slew, 旋向, liftoff height/speed 或 SITL 飞行动作门控。

## 7. 可复现命令（离线环境）

```bash
# 禁止在真实 FC01 上运行；仅在复制的源码目录下执行。
cd 0.4.4-offboard-flight-dev-r5-s10-dev
bash scripts/test.sh  # g++ fixture、原始 C 反例、adapter fixture、Python；非 ROS native
python3 tools/s10_timeline.py path/to/captured_controller.jsonl
# 在有真实 ROS2 Humble 环境的**另一台允许做离线构建的主机**上：
# source /opt/ros/humble/setup.bash
# colcon build --packages-select px4_msgs jikl_fc01_system
```

报告结论：**R5 为待 A 实证 + 待 C 独立复验的 DEV，NO-GO。**
