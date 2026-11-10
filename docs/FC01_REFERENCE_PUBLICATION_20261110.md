# FC01 地面参考与发布周期针对性修复（20261110）

基准提交：`b388f26bdf731dc50edda2f8ce278cfde36c89ea`。本版保留无视觉 RC 人工解锁、1.5 m 定高、原推力参数、R2 离散安全事件锁存、RC 接管/Kill 和 PX4 Failsafe；不重写起飞状态机。本轮不连接、部署或操纵香橙派，不执行解锁或动力起飞。

本报告区分代码证据、合成离线回放与现场事实。用户提供的 `0.16 m / feedback_age=0.8 s`、`0.09 m / heartbeat_commits=0` 是故障后的显示；没有本次故障原始日志，不能将它们当作触发样本，也不能据此断言飞机曾离地或已经取得 Offboard 权限。

## A：地面高度参考与任务恢复

基准 `rc_navigation.hpp` 在 `WaitNavigation` 中执行：

```cpp
if (!s.landed || std::abs(z0_ - s.z) > c.height_tolerance ||
    std::abs(s.vz) > c.ground_vz_tolerance)
  return block("RC_ARMED_GROUND_REFERENCE_NOT_STABLE");
```

它比较的是本次保存的地面参考与当前高度，不比较 EKF 高度的绝对零值。RC 解锁开关先到、PX4 armed 状态后到的配对窗口也会进入这个分支，因此故障名含 `ARMED` 并不证明触发时 `armed=true`。必须另查触发时的 landed、z0/z/vz、有效性、重置计数和源时间戳。

基准在 Prestream/WaitArm 允许未解锁的健康地面样本更新参考，但缺少独立的连续地面参考证明窗口；发生该故障后，其错误名又未列入可恢复任务故障分类。这是可独立复现的恢复阻断。正常已完成任务的 `reprepare_rc()` 原本会重建 Core，并非所有任务都必然沿用旧 z0。

本版增加任务级地面参考证明：

- 未解锁、非 Offboard、Stabilized、有效 landed/at_rest，以及满足既有静止时长和垂直速度限制。
- 姿态/高度/垂直速度及安全反馈健康；RC 来源新鲜且已选择、解锁开关 OFF、非导航档、低油门中杆、Kill 解除。
- local、status、land、RC 和采样时间戳有效；证明窗口不少于既有 `ground_seconds`，至少 3 帧新 local、3 帧新 land 和 2 帧新 status。各源可按独立 DDS 频率推进，不要求每帧 land 都伴随新 status；反复读取同一帧不能完成证明。
- z/vz/姿态重置计数在窗口内一致，垂直反馈一致性与窗口高度变化检查通过。
- 重置、时间戳回退/间断、同一源时间戳对应变化载荷、垂直异常或不可信地面状态使证明失效。可重新证明的地面变更需要新的完整窗口；进程级时间回退锁存不被此入口清除。

证明窗口可与零推力 PRESTREAM 并行；准备阶段用于零目标的候选参考不代表已经取得起飞参考资格，只有自身连续地面窗口完成后才进入 READY。取得证明后，以当前有效地面 z 建立本次 `z0`，相对高度始终是 `z0 - z`，NED 上升为负 z/vz。不修改 PX4 EKF 原始高度，也不增大原高度容差。未解锁且保持可信地面条件时，参考可以随有界、可信的慢漂移更新；出现解锁意图后冻结，不在已解锁或飞行中重新归零。

`RC_ARMED_GROUND_REFERENCE_NOT_STABLE` 可在故障已结束、退出 Offboard、上锁接地、健康多帧窗口和新的安全 RC 周期全部满足后重新准备。新 Core 重新建立地面参考，不把旧任务的相对高度当作地面证明。日志的 `rc_ground_reference_qualified`、`rc_ground_reference_samples`、`rc_ground_reference_seconds` 用于核对本轮证明；候选参考存在不等于 qualified。

回归还将 A 触发后的最新显示改成 0.09 m，验证保存的触发快照仍是原样本；显示变化不能改写历史触发证据。A 的触发快照复用 `armed_ground_evidence_valid`、`armed_ground_failed_mask`（4=landed、8=height、16=vz）、`armed_ground_sample_z0/z/vz/height/up_speed` 与 `armed_ground_status_stamp/land_stamp/local_stamp`，在实际 RC admission 故障点保存；它们不再只能覆盖后续 ArmedGround 阶段。

这一机制修复可复现的地面参考证明和故障恢复缺陷；它不能证明现场 0.16 m 的具体来源。持续漂移、传感器错误、仍在空中或接地证据不可信仍须阻止任务。

## B：最终发布上下文与控制周期

基准的完整错误路径是：

```text
publish_attitude_resampled()
  -> 第一次上下文策略检查
  -> 编码目标
  -> 第二次上下文策略检查
  -> target_sink() 返回
  -> 第三次上下文策略检查：CONTROL_LOOP_STALL
  -> HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL
node catch
  -> PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL
  -> boundary_fault_latched=true / Safety
```

该精确分支中，target sink 已返回，heartbeat sink 尚未运行；不能把 `heartbeat_commits=0` 等同于 target 也没有发布，更不能把 ROS 本地提交等同于 PX4 接收或执行。真实 sink 异常则连是否提交都可能不确定。

基准 `snapshot()` 每次重新执行 DDS graph 查询。无视觉姿态 profile 每次约 21 次 `count_*`，tick 初始快照与一对目标/心跳的三次检查约 84 次，额外命令路径还会增加。第三次检查自己引入的 graph 延迟会计入周期，可能造成 target 提交后才越过 deadline。基准测试能独立重现这个路径，但没有现场耗时日志，不能断言本次必由 graph 延迟引起，也不能把 OS 调度、回调耗时或源时钟变化排除。

此节点使用单线程 executor；`spin_some()` 在 tick 前执行，tick 内和两个 sink 之间没有再次 spin。当前代码证据不支持“两个 sink 之间的回调改写反馈”这一特定解释。

本版每个控制周期先查询一次 graph 计数，同周期未收到变化事件时复用这一组所有权证据；ROS graph event 触发有界刷新，重新核查实际 Topic 的发布者/订阅者计数。刷新期间再次出现变化、刷新预算耗尽或计数发生冲突时拒绝发布，不用旧计数继续授权。与飞控控制无关的 graph 变化若刷新后计数仍符合独占要求，不直接变成永久 Conflict。缓存不跨周期保留权限证明，刷新耗时也计入当前控制 deadline。

三个最终边界检查继续执行，重新检查当前单调时间、反馈/源采样新鲜度、RC/Kill、权限、图变化和阶段。检查本身的耗时仍计入 deadline；预算失效后不补发过期心跳、不推进未完成的任务发布。R2 收取调度和离散事件锁存保持。

新增 `PublicationAttempt` 将检查阶段、拒绝原因、target/heartbeat 是否本地提交、是否发生 sink 异常分开保存。只对明确未出现解锁意图、无命令事务、未加入飞行、实际未解锁/非 Offboard 且目标为零的 RC 准备阶段，授予已知 deadline 故障的恢复资格。故障期间停止任务和输出；后续健康控制周期与新鲜连续安全地面证据全部重新满足后，才消费这一特定锁存并重建准备状态。

诊断记录保留 `control_cycle_id`、`graph_snapshot_cycle/invalidated`、`publication_stage/decision`、`publication_target_committed`、`publication_heartbeat_committed`、`publication_sink_exception` 和三次 `pair_context_snapshots`；快照包含 armed/Offboard、源时间戳、回调/事件水位、control_fresh 与相对高度，配合各阶段耗时区分触发前后。

未知 sink 异常、非零或已进入飞行的部分发布、时间回退、权限/所有权异常，均不通过上述准备阶段入口解除。不得根据错误字符串相似或一张上锁照片将它们自动归类为可恢复。

## 基准合成回放快照

以下结果来自基准提交的独立离线夹具，未连接 ROS 或飞机；它们不是现场照片的数值复原。根目录验证记录应保留对应测试命令和本轮输出。

本版可复现的故障前、触发、后续显示和新任务 READY 快照见 [离线回放数据](data/FC01_REFERENCE_PUBLICATION_20261110_replay.json)，对应 `native/tests/replay_reference_publication.cpp`。例如合成 A 的触发高度为 0.16 m，随后显示变为 0.09 m 时仍保存原触发样本；安全恢复后新参考为当前地面 1.91 m，相对高度为零。合成 B 明确保留 target=1、heartbeat=0 的本地提交证据，再通过完整新地面窗口恢复。

| 合成场景 | 输入与触发状态 | 基准结果 |
| --- | --- | --- |
| 初始静态地面偏置 0.09 m | z=1.91，其余健康地面，未解锁 | z0=1.91，relative_height=0，READY_WAIT_ARM |
| 初始静态地面偏置 0.16 m | z=1.84，其余健康地面，未解锁 | z0=1.84，relative_height=0，READY_WAIT_ARM |
| A：冻结参考后的变化 | z0=2，RC arm ON、armed=false 后 z=1.84 | A 精确错误，relative_height=0.16；60 帧健康地面后仍 restart_allowed=false |
| B：第三上下文过期 | armed=false、offboard=false，第三检查 control_fresh=false | target_commits=1、heartbeat_commits=0；完整 B 错误；60 帧健康地面后仍不能 reprepare |

这说明 0.09/0.16 m 的静态绝对偏置原本就不必触发 A；A 需要分析保存参考与触发样本的变化。两种数值只作为诊断场景，不是任何情况下都允许的安全误差。

## 本轮针对性回归结果

以下为本轮实际执行的软件回归；合成结果不能替代实机证据。

| 场景/证据 | 本轮结果 |
| --- | --- |
| 0.09/0.16 静态偏置、缓慢漂移、参考冻结、重置重新证明 | `test_rc_navigation` 完整回归通过 |
| 时间戳重复/间断/回退、过期反馈、未知/虚假地面证据 | `test_rc_navigation ground_reference` 场景通过 |
| 第一次/第二次/第三次上下文超时；图变化；sink 异常分型 | `test_control_diagnostics` / `test_offline_timing` / `test_r6_partial_sinks` 与实际节点适配器测试通过 |
| 零准备阶段已知超时恢复；持续超时/未知/非零部分发布仍阻断 | `test_rc_timing_recovery` 及发布边界通过；晚阶段超时与周期间长停顿均使恢复窗口失效 |
| 正常任务连续两轮；A 故障结束后新参考/新 RC 周期 | `test_rc_navigation` 完整回归通过，含生产无视觉 1.5 m 流程 |
| Kill/权限异常/估计器异常与 R2 同轮坏后好回归 | `test_receive_events` / `test_authority_runtime` / 实际节点适配器测试通过 |
| 受影响 Python 回归 | 57 项通过，另有 1 项实际节点源码适配器测试通过 |
| 受影响 C++ 回归 | 15 个测试程序通过，另有诊断回放快照 |
| ROS 2 Humble 本机编译 | x86_64 新目录编译链接通过；不是 ARM64 产物 |
| 安装包源码摘要/配置校验 | 结果记录在随包交付的 `.release.json`，不含主机或历史控制器二进制 |
| 本版香橙派全新 ARM64 构建、硬件与动力验证 | 本轮未执行 |

## 独立 ARM64 升级入口

本版安装包为源码包，不含开发机 x86 或历史 ARM64 控制器。版本与摘要以交付 `.sha256`、包内 `FC01_RELEASE.json` 和构建后 `ARM64_BUILD_EVIDENCE.json` 为准。

| 项目 | 本版值 |
| --- | --- |
| 安装包 | `FC01_OrangePi_Navigation_20261110_reference_publication.sh` |
| 校验文件 | `FC01_OrangePi_Navigation_20261110_reference_publication.sh.sha256` |
| 新目录 | `/home/orangepi/FC01_OrangePi_Navigation_20261110_reference_publication` |
| 必须的新标识 | `FC01_REFERENCE_PUBLICATION_V1_20261110` |
| 入口版本 | `20261110.1` |
| 保留旧目录 | `/home/orangepi/FC01_OrangePi_Navigation_20261109_lifecycle` |

沿用 [20261109 生命周期升级说明](FC01_SERVICE_LIFECYCLE_20261109.md) 的 Agent 保留、单实例、持久服务、旧 Socket 和回滚策略；不要照抄旧版本目录或旧标识。U 盘复制本版安装包及校验文件，在香橙派普通用户终端执行：

```bash
set -euo pipefail
read -r -p 'U盘安装包目录: ' fc01_usb_dir
cd -- "$fc01_usb_dir"
sha256sum -c FC01_OrangePi_Navigation_20261110_reference_publication.sh.sha256
mkdir -p "$HOME/FC01_release_packages/20261110_reference_publication"
cp -n -- FC01_OrangePi_Navigation_20261110_reference_publication.sh FC01_OrangePi_Navigation_20261110_reference_publication.sh.sha256 "$HOME/FC01_release_packages/20261110_reference_publication/"
cd "$HOME/FC01_release_packages/20261110_reference_publication"
sha256sum -c FC01_OrangePi_Navigation_20261110_reference_publication.sh.sha256
export FC01_NAVIGATION_INSTALL_DIR="$HOME/FC01_OrangePi_Navigation_20261110_reference_publication"
test ! -e "$FC01_NAVIGATION_INSTALL_DIR"
test ! -L "$FC01_NAVIGATION_INSTALL_DIR"
bash FC01_OrangePi_Navigation_20261110_reference_publication.sh --install-only
cd "$FC01_NAVIGATION_INSTALL_DIR"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
bash navigation_build_fresh.sh 2>&1 | tee "$HOME/FC01_release_packages/20261110_reference_publication/arm64-build.log"
cat FC01_RELEASE.json ARM64_BUILD_EVIDENCE.json
file install/lib/jikl_fc01_system/fc01_system_node
sha256sum install/lib/jikl_fc01_system/fc01_system_node install/lib/jikl_fc01_system/fc01_config_check
```

必须在香橙派原生全新目录完成构建，不复用 `install`、`.build`、完成记录或旧二进制。构建应输出 `FC01_ARM64_FRESH_BUILD_VERIFIED`，ELF 为 AArch64，提交和二进制摘要与证据一致，并包含本版新标识。构建只生成程序，不启动控制器或 Agent。

仅在现场已经上锁接地、控制权释放时切换服务；若旧控制器还在安全退出，不强杀、不启动另一实例。保留 MicroXRCEAgent，不抢占 UART。若存在历史 standby 启动入口，按旧文档先处理其单实例冲突。

```bash
sudo systemctl stop fc01-navigation.service
systemctl show fc01-navigation.service -p ActiveState -p SubState -p MainPID
pgrep -af '[/]fc01_system_node([[:space:]]|$)' || true
cd "$HOME/FC01_OrangePi_Navigation_20261110_reference_publication"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
bash navigation_restore.sh configure --confirm-flight-test --boot-policy enable
```

确认旧 native 已退出且配置器成功。保存本次输出中 `rollback=` 后的完整路径。`configure` 只 enable，不 start；第一次启动由操作者执行，此后已启用服务随系统启动进入安全准备。

```bash
sudo systemctl start fc01-navigation.service
systemctl is-enabled fc01-navigation.service
systemctl cat fc01-navigation.service
python3 navigation.py status
journalctl -u fc01-navigation.service -n 100 --no-pager
```

核对启动记录的 `controller_build=FC01_REFERENCE_PUBLICATION_V1_20261110`、`entry_version=20261110.1`、新目录配置来源、实际 Flight Test 权限及 `0.50/0.04/0.80`、1.5 m、视觉关闭、自动解锁关闭。只允许一个新 native PID，`/proc/<PID>/exe` 路径和摘要须匹配本版 ARM64 证据。服务 active 不等于下一次动力试验条件已满足。

正常任务结束或本版支持的可恢复故障解除后，记录 `rc_safe_reprepare` 的历史原因和任务序号，以及新的地面参考证明、PRESTREAM/READY_WAIT_ARM；不得用进程重启或历史 relative_height=0 替代这些证据。下一次仍要求新的人工 RC 操作。

停止及回滚仍使用本次配置记录的备份：

```bash
sudo systemctl stop fc01-navigation.service
cd "$HOME/FC01_OrangePi_Navigation_20261110_reference_publication"
read -r -p '本次configure的rollback完整路径: ' fc01_rollback_backup
bash navigation_restore.sh rollback --backup "$fc01_rollback_backup"
```

回滚恢复原 unit 及原持久状态，不自动启动旧版本。未覆盖、删除或链接冒充旧目录。

## 尚未取得的现场证据

本轮没有新 ARM64 编译、实机重启、真实 graph 干扰、飞行中断链或动力连续任务证据。此前 ARM64/DDS 成功记录属于旧提交，不能替代本版证据。A 的现场触发样本、B 的实际耗时分解及是否已 Offboard 仍未知；先前 `VERTICAL_VELOCITY_INCONSISTENT` 的飞行根因也不因本次修改而消失。

地面参考与发布周期是两个独立修复：前者加强参考证明并解除已结束 A 故障的软件恢复阻断；后者移除同周期重复 graph 查询并保留最终时效/安全检查。两者都不保证在持续的估计、时序、权限或硬件故障下能够继续起飞。
