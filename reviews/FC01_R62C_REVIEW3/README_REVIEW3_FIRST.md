# FC01 R6.2c Review3：拆桨台架估计漂移宽限

本包由 `FC01_R62C_REVIEW2_20261015(1).zip` 原源码独立修改，解压目录为 `FC01_R62C_REVIEW3`。完整保留源码、px4_msgs 和既有工具。原 Review2 可保留。

## 本次放宽多少

以下为 **Arm-only 的自动 Disarm 地面观察**，高度定义为 `height=z0-z`。

| 条件 | Review2 | Review3 |
|---|---:|---:|
| 向下估计偏移，负 height 的绝对值 | 0.12 m | **0.50 m** |
| 向上估计高度 | 0.12 m | **0.12 m** |
| 垂直速度绝对值 | 0.10 m/s | **0.25 m/s** |
| 新地面样本驻留 | 至少 2 s | 至少 2 s |
| Disarmed 后新的 `at_rest=true` 驻留 | 必须 | 必须 |

参数位于 `config/attitude_altitude.yaml`：

```yaml
aa_bench_ground_down_tolerance_m: 0.50
aa_bench_ground_vz_tolerance_mps: 0.25
```

放宽贯通台架 ArmedGround、PoweredGround、最终 Disarm 出口、停机状态验证和模式释放。普通飞行、普通接地与起飞使用原阈值。所有动作开关在配置文件中默认关闭；台架始终只允许精确零外部推力。

这两项是固定拆桨台架的工程容差，不能据此判断估计变化一定是漂移，或用作有桨飞行参数。

## 1. 解压后只编译一次

下载本 ZIP 后，在普通终端运行：

```bash
ZIP="$(find "$HOME" -maxdepth 5 -type f \
  -name 'FC01_R62C_REVIEW3_20261016*.zip' -print -quit 2>/dev/null)"
if [ -z "$ZIP" ]; then
  echo 'ERROR: 请先下载 Review3 ZIP'
elif [ -e "$HOME/FC01_R62C_REVIEW3" ]; then
  echo 'STOP: Review3 目录已存在，避免覆盖'
else
  unzip -q "$ZIP" -d "$HOME"
fi
```

确认目录是本次新解压的 Review3，然后：

```bash
cd "$HOME/FC01_R62C_REVIEW3" || exit 1
bash scripts/review3_build.sh
```

成功标志为 `Summary: 2 packages finished` 和 `REVIEW3_NATIVE_BUILD_PASS`。该脚本只编译。当前交付环境没有 ROS 2 Humble，离线编译不能替代这一步。

## 2. 编译通过后，单次拆桨闭环

沿用已通过的通信与接管准备。保持 DDS Agent 运行，确认四片桨拆除、机体固定、QGC 为 Disarmed、独立停机手段可用；已有控制节点必须先安全退出。

终端 1：

```bash
cd "$HOME/FC01_R62C_REVIEW3" || exit 1
bash scripts/review3_bench.sh
```

脚本显式运行 `install_review3`，保存独立日志路径到 `.review3_log`。它启用专用台架权限，但启动节点本身不会解锁。

终端 2：

```bash
cd "$HOME/FC01_R62C_REVIEW3" || exit 1
python3 tools/fcctl.py --socket "$(cat .review3_socket)" --bench-arm-only
```

依次输入 `authority rc`、`attitude-ready`。达到 `OFFBOARD_READY` 且 `armed=False` 后，提交一次 `arm-only`，按提示输入 `BENCH ARM CONFIRM`。

预期：

`REQUEST_ARM → BENCH_ARMED_GROUND → POWERED_GROUND_UNCONFIRMED → REQUEST_DISARM → SAFE_DISARMED → RELEASE_OFFBOARD → IDLE`

`BENCH_DISARMED_WAIT_NEW_REST_GROUND_DWELL` 是停机后等待新静止证据。出现异常时使用既有独立停机手段；确认 QGC 为 Disarmed 且电机停止后再退出程序。

## 3. 验收看这一份日志

闭环完成并确认停机后，在普通终端运行：

```bash
cd "$HOME/FC01_R62C_REVIEW3" || exit 1
LOG="$(cat .review3_log)"
grep -E 'command_sink_audit|BENCH_NEW_GROUND_DWELL_DISARM_REQUESTED|BENCH_DISARMED_WAIT_NEW_REST_GROUND_DWELL|SAFE_DISARMED|RELEASE_OFFBOARD|"phase_after":"IDLE"' "$LOG" | tail -35
```

成功需要一次 `intent=ARM`、一次 `intent=DISARM` 的真实本地提交，Disarm 的 `command400_generation=2`，以及新的 PX4 Disarmed 和停机后静止驻留，最终回到 IDLE。本地提交不等于飞控接收；如果由 RC 或 PX4 自行 Disarm，只记录外部停机，不算本程序自主 Disarm 成功。

新增日志包括有效台架容差、当前阶段已累计的独立地面样本驻留时间和首末时间戳。高度/速度/接地越界时，`armed_ground_sample_*` 保存 Core 真正触发故障的采样，不依赖事后重采样。

## 验证与文件

- `REVIEW3_CHANGELOG_AND_VALIDATION_zh.md`：修改、测试和限制。
- `FC01_REVIEW2_to_REVIEW3.patch`：仅用于原 Review2；优先使用完整新目录。
- `evidence/REVIEW3_INITIAL_RED.log`：原故障及其他遗漏的失败复现。
- `evidence/REVIEW3_TARGETED_GREEN.log`：针对性回归。
- `evidence/REVIEW3_FINAL_REGRESSION.log`：完整离线回归。
- `evidence/REVIEW3_CODE_REVIEW.md`：独立审查记录。
- `REVIEW3_SHA256_MANIFEST.json`：本包逐文件校验。Review2 及更早报告、manifest、补丁属于历史记录。
