# FC01 恢复诊断修复第五版：Orange Pi 交付

版本：`20261110.7`。控制器标识：`FC01_RECOVERY_DIAGNOSTIC_V5_20261110`。
目标：Orange Pi Zero 3W、Ubuntu 22.04、AArch64、ROS 2 Humble、PX4 v1.16。
本版源码包含四项复核后的修复，使用独立 V5 安装目录；旧 V4 安装包和目录保留。
[V4 说明](FC01_RECOVERY_DIAGNOSTIC_V4_20261110.md)保留其历史现场分析。

## 本版四项修复

1. **落地反馈变化时保持已有受控输出。** 在已解锁的 `LANDED` / `REQUEST_DISARM` 阶段，
   队列中保留的落地变化仍清除接地证明、阻止不安全 Disarm，并让原有反弹/下降控制继续执行。
   修复此前这条等待分支跳过整轮输出的问题；故障状态和未解锁准备不因此取得输出权限。
2. **垂向控制不因明确无效的水平 NaN 持续停止接收。** 仅在当前无视觉、无 PX4 Land 回退的
   垂向配置中，忽略已明确标为不可用的 x/y、vx/vy、heading 的重复 NaN 对接收调度的影响。
   有效性变化、估计器重置、声称有效却非有限的水平值，以及所有垂向异常仍保留为事件。
   原有接收轮次、预算、源时钟与控制周期保护继续生效。
3. **地面证明窗口保留中途高度偏离。** 准备和故障恢复的地面证明以各自窗口参考为准。
   同周期位置先偏离、后回到参考附近时，早先的偏离仍会否决本轮证明并要求重新观察，
   不能仅用最后一条健康位置延续旧窗口。
4. **状态窗口按实际样本期限撤下旧 READY。** 即使 IPC 仍连接，原生状态停止更新后，
   窗口也按各话题已有消息/采样/接收年龄、原始采样最后推进年龄、`timeout_s` 与经过时间判定反馈是否过期。
   到期撤下 READY 及 CH5/CH6 指导，标为历史；当前控制器的故障和恢复诊断继续显示。
   缺少完整 `source_timing` 的旧格式快照只显示历史，不据此提示手动解锁。

本版没有调整推力、控制增益、1.5 m 目标、RC 导航档位、YAML 配置、消息期限或飞行包线。
人工解锁、随后选择导航、原生落地证明、命令水印和最终发布检查继续生效。
恢复不会自动解锁、自动进入 Offboard 或自动起飞。

PX4 v1.16 的稳定 `vehicle_land_detected` 发布为 1 Hz，关键结果变化时会额外发布；
50 ms 是检测器的备用运行周期，不能等同于稳定消息发布周期。
`at_rest` 恢复需要超过 1 秒未检测到运动。连续每 50 ms 双向翻转的 40 tick 输入属于压力测试，
不作为默认参数下的实机事件序列或本次现场故障证据。旧报告中 `at_rest=false` 的物理原因仍待现场 IMU 证据。

## 香橙派离线解包与构建

把以下两个文件放在香橙派桌面：

- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5.sh`
- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5.sh.sha256`

以普通用户 `orangepi` 运行。飞控可以不接电：

```bash
bash "$HOME/Desktop/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5.sh" offline
```

安装器核对相邻 SHA256、包内清单与文件摘要，解包至
`$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5`，再执行离线构建与配置检查。
同名目录已有不同内容时拒绝覆盖；不要把 `FC01_NAVIGATION_INSTALL_DIR` 指向旧版目录。
重复执行相同包会校验并复用相同源码，构建证据不匹配时使用新的隔离构建目录。

若只需先解包，可运行：

```bash
bash "$HOME/Desktop/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5.sh" --extract-only
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation_deploy.sh" offline
```

`--extract-only` 只解包；第二行才执行构建，两步可分别运行。
默认 SDK 为 `$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE`。
脚本仅在自身进程加载已有 Humble/px4_msgs 环境，执行单任务 ARM64 原生编译、AArch64 ELF、
动态库和实际 YAML 检查，不下载依赖。

离线阶段不运行在线探针，不启动控制器或 Agent，不修改服务和现场运行配置。
`FC01_OFFLINE_READY` 与 `FC01_DEPLOY_COMPLETE` 表示离线阶段完成；正式启用是下一步。
旧服务及原有控制器在离线阶段保持原状。

## 正式启用

现场拆桨，飞控上电，确认未解锁、接地静止、最低油门、其余杆居中、Kill 正常，
CH5 上锁、CH6 位于 Stabilized 非导航档位，实际飞控模式也为 Stabilized。保留既有 MicroXRCEAgent，然后运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation_deploy.sh" activate --confirm-flight-test
```

脚本验证本版离线构建证据并备份原服务、运行配置与开机策略，取得新鲜安全反馈后停止旧服务，
等待旧控制器退出，复用原 Agent，再次确认反馈后配置并启动 V5。
没有既有 Agent、飞控未接通或真实反馈不安全时拒绝启用。
`硬件尚未满足在线启用条件` 与构建失败分别记录，不表示可以跳过安全探针。

启动验收核对本版控制器进程、路径、摘要、工作目录、运行模式、参数来源和新鲜未解锁接地状态。
`FAULT`、`SAFETY` 或停止状态不能验收成功。`FC01_NEW_SERVICE_ACTIVE` 表示服务验收通过，
仍须等待原生实际 `RC_READY_WAIT_ARM`，再重新执行人工解锁与导航顺序。

默认保留 `fc01-navigation.service` 的原开机策略；需要明确启用开机入口时，在启用命令末尾加
`--boot-policy enable`。已有旧 `fc01-standby.service` 的重复入口会先备份、停止并取消启用，
回滚恢复其原策略。旧版文件和安装包不删除。
切换失败会安全收尾；安全条件不足时保留现场与回滚记录，不强杀、不启动第二个控制器，也不自动重启旧控制器。

## 查看状态与保存报告

查看当前说明性状态：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation.sh" status
```

查询 systemd 状态时禁用分页并保留完整路径：

```bash
systemctl --no-pager --full show fc01-navigation.service -p ActiveState -p MainPID -p WorkingDirectory -p UnitFileState
systemctl --no-pager --full status fc01-navigation.service
```

异常后、下一次试验或重启前，立即在普通终端单独导出：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation.sh" report
```

命令无需 SDK，不启停服务，打印 `$HOME/FC01_navigation_report_<时间>.zip` 的完整路径。
不要把它只放在部署命令后面的 `set -e` 脚本中；部署失败可能跳过随后的报告命令。
`navigation.sh status/report` 自身不使用分页；旧终端若停在 `(END)`，按 `q` 退出查看器。

报告包含最新状态、运行配置、controller/standby/install 日志及轮转、探针、服务状态和最近 100 条 journal，
还包含调用版本及实际运行版本的发布/构建/激活证据和最近部署日志（存在时）。
`REPORT_CAPTURE.json` 记录会话身份、文件摘要、缺失、变化、截断及采集开始/结束状态。
日志体积有界，运行中采集不保证原子快照；应尽快保存，避免轮转覆盖。
旧 `probe.json` 不能单独证明当前安全，ZIP 也不包含完整原始反馈或 PX4 ULog。

部署摘要位于 `$HOME/FC01_navigation/install.log`；完整分阶段日志位于
`$HOME/FC01_navigation/deploy-logs/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/`。
在线切换在 V5 目录写入 `ACTIVATION_STATE.json` 并打印 `FC01_ROLLBACK_PATH`。

## 安全回滚

现场拆桨，确认未解锁、接地静止和安全 RC 复位后运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation_deploy.sh" rollback
```

该命令从本版激活记录选择备份。若需要显式指定，使用实际打印出的目录：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5/navigation_deploy.sh" rollback --backup /home/orangepi/FC01_navigation/service-backups/实际记录目录
```

回滚先取得安全反馈并等待当前控制器退出，再取得新鲜地面证据，恢复原服务、运行配置及开机策略。
**旧控制器保持停止**，是否重新启动须现场人工决定。
控制器仍存活、Agent 已变化、安全证据不足或备份/当前服务摘要不匹配时保留现场并拒绝覆盖。
操作中断会进入安全收尾，等待其完成；不要强制结束控制器。

## 验证与适用范围

本版最终开发电脑验证：71 个 C++ 回归程序、365 项 Python 测试（根目录测试 324 项、原生目录测试 41 项）
与 8 项 UBSan 检查全部通过，包含原始采样推进年龄的显示期限检查。
最终冻结副本的正常 8 场景、3058 个控制 tick 输出与基线 `56ad01c` 逐字节一致；
相关回归还覆盖正常多轮任务、起飞/定高/降落及中断恢复。
补充实际节点接口测试使用 1 Hz Land 与 40 Hz local，安全地面恢复至 READY 通过；
真实 socket/子进程覆盖正常 50 ms 更新、200 ms 暂停、较长静默、明确 stale 和 IPC 断开。
这些“实际节点”测试运行真实 C++ 节点路径，ROS API、消息和时钟采用本地测试夹具。

本版开发电脑检查记录于 `HOST_VERIFICATION.json`；安装包校验与重复解包结果记录于
`dist/PACKAGE_VERIFICATION.json`，各项结果以记录中的实际检查状态为准。

`FC01_RELEASE.json` 记录源提交、工作区状态和实际交付文件摘要。基线提交号不能单独证明包内文件相同；
以清单 SHA256、控制器版本标识和实际运行路径联合核对。
`ARM64_BUILD_EVIDENCE.json` 仅在香橙派真实原生构建成功后生成，本包不携带旧二进制或预造 ARM64 成功记录。
旧 V4 包不会因仓库更新自动包含本版修复，已部署服务也不会随源码更新自动切换。

开发电脑通过的测试不等同于香橙派 ARM64/Humble 编译、真实 DDS/UART、systemd 切换或飞行验收。
本版未给推力或姿态作新实机标定，也未定位旧报告中自由飞行姿态变化和 `at_rest=false` 的物理根因。
该类问题仍需对应现场反馈、IMU 记录与 PX4 ULog。
