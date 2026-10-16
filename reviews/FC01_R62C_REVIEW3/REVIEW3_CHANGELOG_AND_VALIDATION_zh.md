# Review3 修改与验证记录

## 交付结论

Review3 已完成本地离线修复和验证，可按 `README_REVIEW3_FIRST.md` 进行新目录原生编译，再由操作者完成一次拆桨闭环验收。离线通过不等于真实 FC01 自动停机已通过。

## 基线与证据

- 基线：`FC01_R62C_REVIEW2_20261015(1).zip`。
- SHA-256：`f0f47a53326a4211bc00241e31ea07ce7662f4554a34a0cedfacf35e5eaabd8a`。
- 用户提供 Review2 故障前后五个 `phase_event` 测值，保存在 `tests/data/review2_reported_five_samples.csv`。本次没有取得完整 Review2 原始日志或新的 ULog。
- 第五个快照中 `height=-0.124492735m`、`vz=0.102222808m/s`；超过旧自动 Disarm 观察的 0.12m、0.10m/s 边界。其余已给出的新鲜度、有效性、Offboard 和 landed 均正常。它不能证明真实机体发生了同等位移，也不能确定估计误差的硬件来源。
- 五个快照约覆盖 0.20 秒，无法单独重建整个驻留计时。实际交付 YAML 中 `aa_land_sample_max_gap=1.25s`，Core 默认的 0.5s 会被其覆盖；本次没有修改地面驻留时长或样本间隔。独立 1Hz 样本的合成回归证明原计时逻辑可以完成驻留。

## 实际修改

1. 增加两个仅用于 Arm-only 的 ROS 参数：`aa_bench_ground_down_tolerance_m=0.50`、`aa_bench_ground_vz_tolerance_mps=0.25`。配置仅允许正有限数，且不可超过本次覆盖的 0.50m／0.25m/s；可以调小。
2. 使用非对称高度窗口：`-0.50m <= height <= min(height_tolerance,liftoff_height_m)`；本包向上界限为 0.12m。Vz 使用绝对值，正负方向均限制为 0.25m/s。
3. 同一台架窗口贯通 BenchArmedGround、PoweredGround、最终 Disarm 发布口、Disarm 状态独立证明、停机后静止确认、SafeDisarmed 和模式释放；台架全局负高度边界也按该窗口检查。普通飞行、普通起飞和普通着陆继续使用原阈值。
4. 保留 landed、三路源数据新鲜与有效、post-Arm 时间戳前进、独立新着陆样本至少两秒驻留、最终出口重新检查、一次普通 Disarm 与停机后的新 at_rest 驻留。没有把零外部推力或单个 landed 布尔值当作独立地面证明。
5. 独立审查发现：外部 Disarm 分支可能提前返回，绕过原单样本 Z/Vz 跳变检查。现将原检查及采样更新移到该分支之前，修复已通过先失败后成功的回归。连续漂移可以容纳，单步 0.49m 异常仍触发 `UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP`。
6. 增加台架故障精确采样、下方高度限制、当前阶段地面驻留进度和首末样本戳；`command_sink_audit` 直接输出 `intent` 与 `command400_generation`。新增独立 Review3 构建和台架启动脚本，明确使用 `install_review3`，每次启动保留独立日志。

## 回归结果

最终命令：`bash scripts/test.sh`。日志：`evidence/REVIEW3_FINAL_REGRESSION.log`。

- 34 个 C++ 回归程序完成；新测试程序包含九组针对性场景。
- Python 共 37 项：35 项通过，2 项因当前执行环境禁止原生 AF_UNIX listener 而跳过。C++ 原生 Socket 分支也有环境性跳过，不能据此声明真实 Socket 传输验收。
- 真实 ROS 适配器对类型/API 夹具的语法编译通过；它不是 ROS 2 Humble 原生编译或链接。
- 新构建/启动脚本通过 `bash -n`；缺 ROS 环境、缺 Review3 二进制时按预期拒绝启动。
- 所有默认动作开关仍关闭。普通飞行的 `test_r62_armed_ground.cpp` 与基线逐字一致且通过。
- 独立审查已复核跳变遗漏的修复，未发现剩余阻塞项。

针对性覆盖：五个真实 z/vz 测值；1Hz 地面消息及连续向下漂移；一次 Arm／一次 Disarm／停机后新静止驻留／回到 IDLE；旧 0.25m 全局下边界和旧 0.15m 退出边界的贯通；向上高度、向下边界、正负速度、接地丢失；land/status/local 冻结；最终出口竞态；不盲目重发；异常配置；外部停机中的异常跳变与连续漂移。

旧台架测试中用于触发 Vz 超限的 0.13749／0.11m/s 改成新界限以外的 0.251m/s；新增正向测试验证原来被拒绝的测值确实获准，普通飞行测试保留原数据。

原 C11/C12 历史反例中的断言中止是现有脚本的预期诊断；脚本会继续验证正向安全属性。这些历史反例不计作新增回归失败。

## 解释边界

五个测值回归只验证这次观察不会再被旧阈值拒绝。完整时间过程、其他健康条件以及停机响应来自显式合成夹具，不能伪装成真机日志回放或 PX4 执行证明。

`bench_ground_dwell_s` 和首末样本戳表示当前阶段；阶段转换后计时器清零，因此成功转换事件中可能显示 0。它不是累计停机时间，也不能单独作为闭环通过证据。

当前环境没有 ROS 2 Humble 或 FC01 连接。没有重新编译真实 ROS 软件、运行 DDS、改变 PX4 参数、刷写固件或执行真实解锁/停机。新的 0.50m／0.25m/s 是台架工程选择，尚未用新硬件闭环证明合适。电池、IMU 与悬停推力没有加入本轮修复范围。

## 官方接口依据

- [PX4 v1.16 VehicleLocalPosition](https://docs.px4.io/v1.16/en/msg_docs/VehicleLocalPosition)：z、vz 使用 NED 向下正方向。
- [PX4 v1.16 VehicleLandDetected](https://docs.px4.io/v1.16/en/msg_docs/VehicleLandDetected)：landed 与 at_rest 为独立字段。
- [PX4 v1.16.0 LandDetector.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/land_detector/LandDetector.cpp)：包内已有上游参考文件，稳定状态按约 1Hz 或检测状态变化时发布。
- [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)：持续 Offboard 存活信号与失联处理。原故障退出策略保持；没有以无条件心跳掩盖故障。
