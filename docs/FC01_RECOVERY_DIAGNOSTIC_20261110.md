# FC01 恢复诊断修复版：Orange Pi 交付

版本：`20261110.3`。控制器标识：`FC01_RECOVERY_DIAGNOSTIC_V1_20261110`。
目标：Orange Pi Zero 3W、Ubuntu 22.04、AArch64、ROS 2 Humble、PX4 v1.16。

本版修复人工接管后的重新准备：新的有效反馈、落地上锁、静止和安全 RC 复位满足后，
可在同一控制器进程内重新进入 `RC_READY_WAIT_ARM`。健康队列事件以及不同频率的
status、land、local 反馈不再无端重置恢复窗口；真实不安全事件仍会否决恢复。
执行完整性故障、未决命令和 ACK 水印继续保留，重新准备不会自动解锁或自动起飞。
首次故障、最新故障、当前恢复阻塞原因、窗口时间和各类新样本数量会输出到状态及日志。

本版继续将 `vz/z_deriv` 速度差作为独立诊断；真实位置残差、无效数据、时间戳、估计器重置、
飞行包线、Kill 和控制权保护仍生效。推力、增益、1.5 m 目标、RC 档位及 YAML 配置没有调整。
旧版说明见 [20261110.2 速度差独立诊断版](FC01_VERTICAL_DIAGNOSTIC_20261110.md)。

## 离线部署

把两个文件放在香橙派桌面：

- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic.sh`
- `FC01_OrangePi_Navigation_20261110_recovery_diagnostic.sh.sha256`

以普通用户 `orangepi` 运行，飞控可以不接电：

```bash
bash "$HOME/Desktop/FC01_OrangePi_Navigation_20261110_recovery_diagnostic.sh" offline
```

脚本校验并解包到独立目录
`$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic`，保留已经解包的旧版本目录。
不要通过 `FC01_NAVIGATION_INSTALL_DIR` 指向旧目录；已有不同内容时会拒绝覆盖。
重复执行相同安装包会校验并复用相同源码；源码、依赖或产物变化时使用新的隔离构建目录。

脚本识别已有 Humble/px4_msgs SDK，执行 ARM64 原生构建、AArch64 ELF/动态库检查和实际 YAML 检查。
默认 SDK 为 `$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE`。
环境仅对脚本生效。离线阶段不运行在线探针、不启动控制器或 Agent、不修改服务与现场运行配置。
`FC01_OFFLINE_READY` 和 `FC01_DEPLOY_COMPLETE` 表示离线阶段完成，正式启用另行执行。

## 正式启用

现场拆桨，飞控上电且未解锁、接地静止，遥控器保持安全开关和非导航档位，然后运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic/navigation_deploy.sh" activate --confirm-flight-test
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
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic/navigation.sh" report
```

命令无需 SDK，不启停服务，输出 `$HOME/FC01_navigation_report_<时间>.zip` 的完整路径。
ZIP 包含最新 `status.json`、运行配置、controller/standby/install 日志及两份轮转、探针、服务状态和最近 100 条 journal。
controller 日志约保留 12 MiB，standby 约 3 MiB；`status.json` 只保留最新快照。
应尽快保存，以免后续运行覆盖故障时间段。异步日志可能丢弃，状态中的 `native_log_dropped` 可帮助判断。
该报告不包含完整原始反馈、PX4 ULog、新版构建/激活清单或 `deploy-logs` 子目录。

状态窗口会分别显示首次故障、最新故障和恢复等待原因；具体的 Runtime 准入阻塞优先显示，
普通恢复等待显示地面窗口的具体条件。正常待命且没有故障记录时不显示恢复区。
过期反馈明确标为历史，窗口时间和样本数不能独立作为下一次解锁的许可。
正常重新准备仍需等到实际 `RC_READY_WAIT_ARM`，并重新执行原有人工授权顺序。

部署摘要位于 `$HOME/FC01_navigation/install.log`，完整分阶段日志位于
`$HOME/FC01_navigation/deploy-logs/FC01_OrangePi_Navigation_20261110_recovery_diagnostic/`。
在线切换还会在本版目录写入 `ACTIVATION_STATE.json`，并打印 `FC01_ROLLBACK_PATH`。

## 安全回滚

现场拆桨、未解锁接地，运行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_recovery_diagnostic/navigation_deploy.sh" rollback
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
