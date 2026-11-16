# 阶段 3：真实 Position 对照日志可用性复核

复核时间：2026-11-16 11:39 UTC（北京时间 19:39）。仅本机文件读取；未启动 ROS、服务或硬件，未改生产源码、参数、串口和发行身份，未提交。

## 结论

用户给出的现场基线是“MTF-02P 已直连 PX4，Position 高度已稳定”。该现场事实继续作为后续同配置对照前提。本次结论只针对**当前可读取的本机存档**：尚无足以量化同配置、真实硬件、空中稳定 Position 高度的 ULog 片段；不把“本地档案中没有”扩大成“用户没有稳定飞过”或“该硬件不可能稳定”。

截至本次复核，阶段 1 保留清单没有新增路径、删去路径或新增内容哈希，也没有保留文件的 mtime 晚于旧清单。30 个唯一内容中，28 个元数据 `ver_hw=DAMIAO_DM_FC01`、2 个 `PX4_SITL`。28 个硬件记录均未见 `vehicle_status[_v1].nav_state=2` 的 Position 区间。现有硬件日志可研究异常，但不能冒充稳定 Position 对照。

唯一有 Position 区间的内容仍是 `FC01_M1_S10_failure.ulg`，SHA-256 `21397a1b4e313a63a6b8401f5632218aa785a0c4212a88dbe8a30612a0b4b9d1`，元数据明确为 `PX4_SITL`。区间 `[155.632,160.636)` 秒，10 条状态样本均 failsafe，且包含落地阶段；不能用其很小的 z 范围证明真实 MTF 悬停。已有完整解读见仓库 `docs/investigations/20261116_stage1/evidence/ulog_extra/README.md`。

最近直接路径硬件记录 `/home/jikl62/桌面/log_10_2026-11-15-16-47-56.ulg` 内容哈希 `8d510d25fbea1e6473939765532ca051705f18cbf967a740fbe05f992490ee11`，仅见 nav_state=15；文件日期不能单独建立某次任务身份。它不含 Position 基线。

## 清单范围与复核证据

依照现有脚本重新发现与只解析状态主题：

```bash
python3 docs/investigations/20261116_stage1/repro/inventory_ulogs.py \
  --output /tmp/fc01-stage3-position-recheck
```

退出 0，摘要 `unique=30 Position-bearing=1`。发现命令为 `rg --files --hidden /home/jikl62 -g '*.ulg' -g '!.git' -g '!node_modules'`，stderr 为空；361 个路径，300 个回收站/PX4 示例排除，保留 61 个路径，30 个唯一 SHA。本次没有重新完整解析全部主题。

- 旧清单：`docs/investigations/20261116_stage1/evidence/ULOG_INVENTORY_RECHECK.json`。
- 新清单：`/tmp/fc01-stage3-position-recheck/ULOG_INVENTORY.json`，与旧清单**逐字节相同**，两者 SHA-256 均 `b2937c1dd164c06c76f48debcbdade2f4e4b8b615300cd050177abc2fa1013ad`。
- 新状态分类：`/tmp/fc01-stage3-position-recheck/ULOG_EXTRA_CLASSIFICATION.json`，SHA-256 `552bbeb0219c87c647e3960ecaaff64b260e76a0b1af02dd7e176ada4e9a4893`。按 SHA 比较旧分类，nav_states、Position 区间、固件及解析错误字段完全相同。分类文本因补列 hardware 元数据及一个重复内容的代表路径不同而变化，不当作新飞行内容。
- 复核脚本 SHA-256 `5e1681bf0dac554c927d2379eb3462a3bbfaf45ba8b453e56622d6e5a9174631`；pyulog 1.2.3。

范围仅为 `/home/jikl62` 中未压缩的保留 `.ulg`。本次未遍历压缩包内部、外接盘、PX4 SD 卡或飞控文件系统。已有阶段 1 桌面 ZIP 目录索引也不等同全机所有归档内容。文件 mtime 与文件名都不能证明真实飞行时间。

## 下一份可比 Position 记录需要的身份与区间

应使用用户已稳定的原硬件接线、固件、PX4 参数及传感器安装配置，保存同一配置的真实 Position ULog，覆盖解锁前静止地面、起飞、明确的空中 Position 稳态窗、降落和上锁。先查内容身份与状态，再选窗口；不能只依文件名、nav_state=2 或一段小 z 范围宣称悬停稳定。记录完整固件哈希、硬件/UUID、初始参数和运行中的参数变化、dropout、主题实例与采样率。若与 companion 异常任务比较，还须绑定该任务同次 ULog、companion READY 前开始的连续日志、boot/PID、运行二进制哈希、runtime 和源码发行身份。

Position 区间至少联读：`vehicle_status[_v1]` 的 timestamp/nav_state/arming_state/failsafe，以及 `vehicle_land_detected` 的 landed/maybe_landed/ground_contact。有效位置/速度、非接地、解锁、无 reset 或 dropout 的连续空中窗分别报告；窗口若不足或资料缺字段，明确为资料不足，不能补成稳定证据。

## 必要主题、字段与可复核指标

| 用途 | 主题/字段 | 指标与解释 |
| --- | --- | --- |
| 地面参考与相对高度 | `vehicle_local_position`: timestamp、timestamp_sample、z、vz、z_deriv、z_valid、v_z_valid、z_reset_counter、vz_reset_counter、delta_z、delta_vz | 每个连续坐标段单独建立静止地面 z0，`h=z0-z`，上正速度 `-vz`。报告样本数/时长、h 均值/中位数/标准差/峰峰值、vz 的 RMS/分位数；若没有可核实目标值，不报“跟踪误差”。有 reset 时不能跨段沿用旧 z0。 |
| 反馈连续性 | 同上，加 local 的所有实际记录的精度/有效位 | 报告采样间隔中位数/高分位/最大 gap、无效和非有限计数、reset 时刻；比较 `vz-z_deriv` 分布及 z 增量与导数积分残差。ULog 降采样不能还原完整 DDS 接收时序，也不能由此断言实际节点 TTL 通过。 |
| 测距与设备身份 | `vehicle_local_position`: dist_bottom、dist_bottom_valid、dist_bottom_variance、dist_bottom_sensor_bitfield、dist_bottom_reset_counter（按真实字段）；`distance_sensor` 每个实例的 timestamp、device_id、orientation、current_distance、min/max_distance、signal_quality/variance（有则保留） | 先核实下视设备与新鲜有效范围。报告 raw range 与 PX4 dist_bottom 的有效率、噪声/阶跃、clip/缺测，以及同窗趋势。dist_bottom 是 PX4 底面/地形距离估计，不是已认证的 MTF 原始量，也不保证独立于 EKF z；单看 z 与 dist_bottom 同向不构成独立物理验证。斜视、姿态与地面变化需说明。 |
| 实际融合来源 | `estimator_status_flags` 的 cs_rng_hgt/cs_rng_terrain/cs_opt_flow/cs_baro_hgt/cs_gps_hgt/EV 相关位；`estimator_aid_src_rng_hgt`、baro/GNSS/EV 对应 aid-source 实例，及 optical flow/sensor_optical_flow 实例 | 报告实际控制位、fused/innovation_rejected/innovation/test_ratio（按实际主题存在情况）与启停时间。参数“启用”不等于正在融合；主题缺失写未记录，不能据此一概判为传感器不存在。 |
| 姿态与 reset | `vehicle_attitude`: timestamp/sample、q、quat_reset_counter、delta_q_reset；需要时 estimator reset 相关主题 | 对齐 q/reset、z/vz reset 与异常首因，不将相同 q、正常后续帧当作 reset 参考连续证明。 |
| 控制与推力 | `vehicle_local_position_setpoint`、`trajectory_setpoint`、`vehicle_attitude_setpoint`、`vehicle_thrust_setpoint`、`actuator_motors`/实际记录主题、hover thrust estimate | 对 Position 的目标/估计/推力连续性做上下文核对。不同固件或任务可能记录不同主题，不能把 companion 未收到与 PX4 未记录混为同一事实。 |
| 状态/失联/命令 | `offboard_control_mode`、`vehicle_control_mode`、`failsafe_flags`、`vehicle_command`/ack、事件与 console | 对异常任务按最后实际收到心跳、模式变化、failsafe、降落/上锁的顺序解释；sink 返回不等于 PX4 实收。保留 COM_OF_LOSS_T/COM_OBL_RC_ACT 等当次值，不能反推唯一根因。 |

基线 h 稳态统计说明的是**估计高度重复性**。没有独立物理参考时，不能把小方差称作真实高度精度。对比 Position 和 companion 时必须使用相同符号、相对地面参考及明确窗口；不能直接比较不同飞行中的绝对 local.z。

companion 侧至少保存 `phase_event`/连续状态中的 monotonic 时间、源戳、height_reference_z0、height_sample_z、relative_height_m、velocity_ned、rc_target_height、takeoff_stage、liftoff_*、target_height_settled、目标/推力、vertical_feedback_*、range_*、安全首因与真实发布计数。`range_compared` 是最近已完成至少 200ms 区间结果，`range_sample_stamp` 是最新样本，不能把每条日志当作新差分。ULog PX4 boot 微秒和 companion host monotonic/ROS 域须以可核实事件或时钟记录对齐；不能直接相减。
