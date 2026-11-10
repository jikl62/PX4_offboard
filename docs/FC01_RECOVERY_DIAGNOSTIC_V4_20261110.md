# FC01 恢复诊断修复第四版：Orange Pi 交付

版本：`20261110.6`。控制器标识：`FC01_RECOVERY_DIAGNOSTIC_V4_20261110`。
目标：Orange Pi Zero 3W、Ubuntu 22.04、AArch64、ROS 2 Humble、PX4 v1.16。


## 本版新增修复：故障后接收积压

用户提供的 `FC01_navigation_report_20261021_152318_747738.zip` 确认正在运行第二版 `.4`。
本次最初停止发布由 `KILL_OR_TERMINATION` 触发，随后以 `OPERATOR_TAKEOVER_LATCHED` 保留首次故障。
位置和姿态消息之后逐渐积压；最终位置样本约旧 0.537 秒，超过原有 0.5 秒期限，
其中约 0.497 秒在中间件接收至回调之间。日志没有本次运行时钟跳变证据。

已用真实节点回调复现两个会阻止追读的路径：故障后仍使用旧任务高度参考判断“立即控制”，
或位置载荷的水平分量为 NaN，每次消息都结束额外接收轮次。
20 Hz 控制循环于是只能每周期消费约一个样本，已有队列难以清空，持续读到过期数据。

本版仅在 `SAFETY` 且本地输出已经停止时，允许在原有接收预算内继续追读。
所有离散/不安全载荷仍保留并在本周期检查；任何不安全载荷都会否决本周期恢复。
同周期的坏时间戳、非 Stabilized 状态及来源身份冲突也分别保留，不能借用后续正常消息的有效性。
`receive_timing_block_source` / `receive_timing_block_reason` 输出本周期首个接收时序阻塞事实（存在时）。
原有最多 10 轮、协作式 5 ms 预算、队列溢出和时钟保护保留，消息期限仍为 0.5 秒。
完整单轮回调不可抢占，5 ms 不是硬实时上限；原控制周期保护继续覆盖实际耗时。
准备、待解锁和活动飞行继续按原事件停止规则运行。

这份报告末段飞控同时持续报告 `at_rest=false`，静止时间为零，是独立的恢复阻塞。
修复接收积压不会替代飞控接地静止证明，也不能保证该现场立即恢复。
详细分析单独交付于 `dist/TEST_REPORT_ANALYSIS_20261110_152318_zh.md`。

状态窗口会同时解释具体过期来源、消息年龄和接地反馈中的静止条件，并明确标注这些观测的快照年龄。
这些是采集当时的观测，不代表当前安全许可；仍须以原生状态机的实际 READY 为准。
原生状态新增只读 `local_payload_z_valid`、`local_payload_vz_valid` 和 `px4_user_takeover_reported`，
用于区分原始载荷标志与已有的过期后有效性屏蔽。原始载荷也可能过期，必须结合各源时间戳/新鲜度判断；
这些诊断不参与准入，不改变已有字段含义。

## 地面仍不静止时的 PX4 只读取证

PX4 v1.16 官方落地检测会在 `landed=true` 且超过 1 秒未记录运动后报告 `at_rest=true`。
会刷新最后运动时刻的条件包括三轴角速度模长超过 3°/s（含 yaw）、所选 IMU 的
`gyro_vibration_metric > 0.02` 或 `accel_vibration_metric > 1.2`。
这些振动指标不是静态重力大小。`LNDMC_ROT_MAX` 属于另一项落地判断，不能代替静止门限。
未发现 Kill 后必须重启才能清除 `at_rest` 的官方实现；本报告没有足够 IMU 证据说明具体物理原因。

在 QGC 的 MAVLink Console 读取以下内容；这些命令不是香橙派 bash 命令：

```text
listener vehicle_land_detected -n 5 -r 2
listener sensor_selection -n 1
listener vehicle_angular_velocity -n 20 -r 10
listener vehicle_imu_status -i 0 -n 10 -r 2
```

`vehicle_imu_status` 的实例应与 `sensor_selection.gyro_device_id` 匹配；如不匹配，
对已有的其他实例（`-i 1`、`-i 2`、`-i 3`）读取并核对 `gyro_device_id`。
短时间低值采样可能漏掉瞬态运动，不能据此直接取消静止保护或调整门限。
这些只读命令在本次开发环境未连接飞控执行，具体原因仍待现场证据。

## 继承的修复：BEFORE_READY 后安全重试

`RC_ARM_SOURCE_BEFORE_READY` 和 `RC_ARM_STATE_BEFORE_READY` 分别表示解锁开关样本或解锁状态消息
没有越过 READY 的源时间戳边界。旧版将它们作为必须重启的故障；即使随后安全复位也不能重新准备。
本版保持对早到/迟到旧授权的拒绝，取消本轮尝试后，允许通过完整的新鲜地面证据窗口重新准备。
原有的时间戳边界、人工解锁和后续导航顺序继续生效；旧解锁证据不能用于新任务。

`RC_ARM_OR_NAVIGATION_BEFORE_READY` 是在预发送尚未完成时提前解锁、提前选导航或已在 Offboard。
它原本已有安全复位重试通路，本版也回归检查这条通路。
此前 BEFORE_READY 反馈只保留了部署成功输出，未保留完整故障英文和报告，因此不能断言当时命中了哪一种。
这两项源码缺陷已有独立复现；部署成功也不代表任务已经 READY。

发生准备中断后：确认飞机已落地上锁，CH5 OFF、CH6 Stabilized、Kill 正常、最低油门且其余杆居中，
等待有效数据和地面证明，再等待实际 `RC_READY_WAIT_ARM`。随后重新手动解锁，再选择导航。
恢复期间保持上述安全状态；持续拨动开关会使条件不连续。仍等待时先导出报告，关注
`rc_first_fault_reason`、`rc_latest_fault_reason`、`rc_recovery_block_reason` 和 `rc_reprepare_block_reason`。
无需为了取消这两种准备失败而给香橙派重新上电。


若显示 `OPERATOR_TAKEOVER_LATCHED`，它是本轮首次接管故障，恢复等待期间保留。
仅看这个名称不能知道当前阻塞项；以“恢复等待原因”和最新故障、实际 armed/模式/接地反馈为准。
本版根据本次报告修复接收积压；仍须取得实际接地静止和新鲜安全反馈才能重新准备。
PX4 v1.16 官方实现会在实际上锁后清除 `failsafe_and_user_took_over`，落地但仍解锁不等于该位已清。
因此不能通过忽略该字段来推定安全；状态不符时先保留报告，核对实际飞控反馈。

本版修复人工接管后的重新准备：新的有效反馈、落地上锁、静止和安全 RC 复位满足后，
可在同一控制器进程内重新进入 `RC_READY_WAIT_ARM`。健康队列事件以及不同频率的
status、land、local 反馈不再无端重置恢复窗口；真实不安全事件仍会否决恢复。
执行完整性故障、未决命令和 ACK 水印继续保留，重新准备不会自动解锁或自动起飞。
首次故障、最新故障、当前恢复阻塞原因、窗口时间和各类新样本数量会输出到状态及日志。
本版还修复：新鲜 IPC 的故障不会被飞控反馈过期遮成 WAITING；停止发布事件记录真实解锁/模式/接地状态；
同周期保留首个恢复阻塞事件；恢复时钟回退重开证明；二次故障以最近已观察样本建立新的因果基线。
准备阶段显示未静止、静止不足、档位、遥控和数据前提等具体观测，帮助定位等待原因。

本版继续将 `vz/z_deriv` 速度差作为独立诊断；真实位置残差、无效数据、时间戳、估计器重置、
飞行包线、Kill 和控制权保护仍生效。推力、增益、1.5 m 目标、RC 档位及 YAML 配置没有调整。
旧版说明见 [20261110.2 速度差独立诊断版](FC01_VERTICAL_DIAGNOSTIC_20261110.md)。

## 离线部署

把两个文件放在香橙派桌面：

- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4.sh`
- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4.sh.sha256`

以普通用户 `orangepi` 运行，飞控可以不接电：

```bash
bash "$HOME/Desktop/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4.sh" offline
```

脚本校验并解包到独立目录
`$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4`，保留已经解包的旧版本目录。
不要通过 `FC01_NAVIGATION_INSTALL_DIR` 指向旧目录；已有不同内容时会拒绝覆盖。
重复执行相同安装包会校验并复用相同源码；源码、依赖或产物变化时使用新的隔离构建目录。

脚本识别已有 Humble/px4_msgs SDK，执行 ARM64 原生构建、AArch64 ELF/动态库检查和实际 YAML 检查。
默认 SDK 为 `$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE`。
环境仅对脚本生效。离线阶段不运行在线探针、不启动控制器或 Agent、不修改服务与现场运行配置。
`FC01_OFFLINE_READY` 和 `FC01_DEPLOY_COMPLETE` 表示离线阶段完成，正式启用另行执行。

## 正式启用

现场拆桨，飞控上电且未解锁、接地静止，遥控器保持安全开关和非导航档位，然后运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4/navigation_deploy.sh" activate --confirm-flight-test
```

脚本验证本版离线构建证据，备份原服务及运行配置，取得新鲜安全反馈后停止旧服务并等待控制器退出，
保留既有 MicroXRCEAgent，重新探测后配置并启动新版。没有既有 Agent 或反馈不安全时拒绝启用。
飞控未连接会报告“硬件尚未满足在线启用条件”，这与编译失败分别记录。
启动验收同时核对本版进程、路径、摘要、运行模式、参数来源和新鲜未解锁接地状态；
`FAULT`、`SAFETY` 或停止状态不能验收为成功。服务按原人工 RC 授权流程准备。

默认保留原开机策略。需要启用开机入口时显式添加 `--boot-policy enable`。
若存在旧 `fc01-standby.service` 的重复开机入口，切换会备份并停止该入口，回滚恢复原策略。

## 测试后保存状态

出现异常后、再次试验或重启前，立即执行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4/navigation.sh" report
```

命令无需 SDK，不启停服务，输出 `$HOME/FC01_navigation_report_<时间>.zip` 的完整路径。

查看服务请禁用分页并显示完整路径；`(END)` 是查看器提示，按 `q` 可退出：

```bash
systemctl --no-pager --full show fc01-navigation.service -p ActiveState -p MainPID -p WorkingDirectory
```

如果将部署和报告写在同一个 `set -e` 脚本里，部署失败会使后面的报告命令跳过。
异常时请在普通终端单独执行上面的 `report` 命令；不要先重启清空现场。

ZIP 包含最新 `status.json`、运行配置、controller/standby/install 日志及两份轮转、探针、服务状态和最近 100 条 journal。
controller 日志约保留 12 MiB，standby 约 3 MiB；`status.json` 只保留最新快照。
应尽快保存，以免后续运行覆盖故障时间段。异步日志可能丢弃，状态中的 `native_log_dropped` 可帮助判断。
报告还包含调用版本与实际运行版本的发布、构建和激活证据（存在时），各自最近 6 份部署日志，
以及 `REPORT_CAPTURE.json` 中的会话身份、采集摘要、截断/缺失/变化说明和文件 SHA256。
导出有体积上限；超限日志保留尾部并明确标记。运行期间采集不保证原子快照，附开始/结束状态供核对。
`probe.json` 可能来自旧启动，不能单独当作当前安全证据。报告不包含完整原始反馈或 PX4 ULog。
新版控制器日志按完整记录保存；正常 JSON 不再被 4096 字节管道分块插入时间戳，超长输出仍有界分段并明确标记。

状态窗口会分别显示首次故障、最新故障和恢复等待原因；具体的 Runtime 准入阻塞优先显示，
普通恢复等待显示地面窗口的具体条件。正常待命且没有故障记录时不显示恢复区。
过期反馈明确标为历史，窗口时间和样本数不能独立作为下一次解锁的许可。
正常重新准备仍需等到实际 `RC_READY_WAIT_ARM`，并重新执行原有人工授权顺序。

部署摘要位于 `$HOME/FC01_navigation/install.log`，完整分阶段日志位于
`$HOME/FC01_navigation/deploy-logs/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4/`。
在线切换还会在本版目录写入 `ACTIVATION_STATE.json`，并打印 `FC01_ROLLBACK_PATH`。

## 安全回滚

现场拆桨、未解锁接地，运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v4/navigation_deploy.sh" rollback
```

回滚先确认安全反馈并等待当前控制器退出，随后重新取得新鲜地面证据，再恢复原服务、运行配置及开机策略。
旧控制器保持停止。控制器仍存活或安全证据不足时保留现场与备份，拒绝覆盖，不强杀或启动第二个实例。
可用 `--backup /home/orangepi/FC01_navigation/service-backups/实际记录目录` 显式选择备份。
操作中断会进入安全收尾，等待完成；不要连续发送中断或强制结束控制器。

## 源码和验证范围

`FC01_RELEASE.json` 记录基线提交、工作区状态和实际交付文件的 SHA256。
提交号只标识基线；本包含未提交修复，并保留原有 `fc01_config_check.cpp` 内容。
`HOST_VERIFICATION.json` 记录本版开发电脑检查。
`ARM64_BUILD_EVIDENCE.json` 只在目标设备真实原生构建成功后生成，不预先携带 ARM64 成功记录或旧控制器二进制。
开发电脑回归通过不等于已完成 Orange Pi 编译、DDS、实际服务切换、恢复或飞行验证。
本版没有调整飞行增益、推力、目标高度或包线阈值，也没有诊断出旧报告中自由飞行姿态严重变化的根因。
地面恢复修复不能替代该次飞行对应 PX4 ULog 的控制/电机输出分析。
