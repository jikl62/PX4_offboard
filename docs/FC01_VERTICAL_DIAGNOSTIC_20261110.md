# FC01 速度差独立诊断版：Orange Pi 交付

版本：`20261110.2`。控制器标识：`FC01_VERTICAL_DIAGNOSTIC_V1_20261110`。

目标：Orange Pi Zero 3W、Ubuntu 22.04、AArch64、ROS 2 Humble、PX4 v1.16。
本包含当前工作区的真实源码，包括未提交的垂直反馈修复和原有 `fc01_config_check.cpp` 内容。
提交号只表示工作区基线；逐文件 SHA256 才是本包源码身份。
不携带开发电脑或历史版本的控制器二进制。

## 从 U 盘部署

把以下两个文件放在 `/home/orangepi/Desktop`：

- `FC01_OrangePi_Navigation_20261110_vertical_diagnostic.sh`
- `FC01_OrangePi_Navigation_20261110_vertical_diagnostic.sh.sha256`

以普通用户 `orangepi` 打开终端。飞控可以不接电，执行唯一命令：

```bash
bash "$HOME/Desktop/FC01_OrangePi_Navigation_20261110_vertical_diagnostic.sh" offline
```

脚本自动校验、解包到独立目录
`/home/orangepi/FC01_OrangePi_Navigation_20261110_vertical_diagnostic`，
识别既有 SDK、原生编译、检查 AArch64 ELF 和动态库、校验真实 Flight Test 配置，
生成 `ARM64_BUILD_EVIDENCE.json` 与 `OFFLINE_DEPLOYMENT.json`。
SDK 默认路径为
`/home/orangepi/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE`。
环境只对脚本生效，不改 `.bashrc` 或全局环境。

离线部署不运行在线探针、不要求 DDS 话题、不启动控制器或 Agent、不修改服务及正在使用的配置。
看到 `FC01_OFFLINE_READY` 和 `FC01_DEPLOY_COMPLETE` 才表示离线阶段完成；这不表示正式启用。
重复执行校验全部包内文件；相同解包不重写，同源码与依赖且产物证据完整时复用编译。
依赖或产物变化时使用新的隔离构建目录；失败保留日志和旧目录，不借用旧二进制。

## 正式启用

现场拆桨、飞控上电、未解锁、接地静止，遥控器打开并处于安全开关及非导航档位。
本步骤保留原安全 probe，并额外检查真实 RC 安全反馈。
执行唯一命令：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_vertical_diagnostic/navigation_deploy.sh" activate --confirm-flight-test
```

脚本核对离线证据，识别旧服务，取得新鲜安全反馈后停止旧服务并等待控制器退出，
保留既有 MicroXRCEAgent，重新 probe，再备份和配置新版、启动并核对进程/路径/摘要/新鲜状态。
Agent 缺失时拒绝启用，部署探针不新建 Agent。
飞控未连接或反馈不安全时显示“硬件尚未满足在线启用条件”，不能视为编译失败。
不会发送解锁、Offboard 或起飞命令；服务从原 RC 人工解锁准备流程开始。
`active` 本身不能证明验收成功，脚本还需确认本版控制器、新鲜状态和 `diagnostic_only` 字段。

默认保留原开机策略；若需要明确启用开机待命，可在上述命令末尾加 `--boot-policy enable`。
已有旧 `fc01-standby.service` 的重复开机入口会在安全切换中备份、停止并取消，回滚时恢复原策略。

## 日志和安全回滚

每一步显示完整日志路径。摘要日志固定为：

```bash
tail -n 60 "$HOME/FC01_navigation/install.log"
```

完整分阶段日志位于
`$HOME/FC01_navigation/deploy-logs/FC01_OrangePi_Navigation_20261110_vertical_diagnostic/`。
原有构建/探针细节也保留在安装日志及 `probe.json`，不会把无 DDS 反馈归咎于编译。
不因为无关 Netplan 警告修改网络或系统权限；依据命令返回值及本服务结果判断。

正式切换输出 `FC01_ROLLBACK_PATH=...`，并写入新版目录的 `ACTIVATION_STATE.json`。
原 unit、运行配置和开机状态备份位于 `$HOME/FC01_navigation/service-backups/`。
在拆桨、安全接地状态下执行：

```bash
bash "$HOME/FC01_OrangePi_Navigation_20261110_vertical_diagnostic/navigation_deploy.sh" rollback
```

回滚要求真实安全反馈，等待新版安全退出后恢复配置和原开机策略，**不自动启动旧控制器**。
若控制器仍存活或安全证据不足，脚本拒绝覆盖并保留现场；不要强杀或启动第二个实例。
若需要显式选择备份，在命令末尾加 `--backup /home/orangepi/FC01_navigation/service-backups/实际记录目录`。
安装/切换失败不会自动拉起旧控制器。旧版本目录与已有 Agent 都保留。

## 源码与验收范围

`FC01_RELEASE.json` 记录版本、基线提交、工作区状态及每个交付文件的 SHA256。
`HOST_VERIFICATION.json` 记录开发电脑实际执行的验证及其限制。
`ARM64_BUILD_EVIDENCE.json` 只在目标设备成功完成真实原生构建后生成，记录编译器、SDK、ROS、CMake、
源码、动态库、可执行文件与 AArch64 ELF 证据；包内不预制 ARM64 成功记录。

持续 `vz/z_deriv` 差异只更新诊断，不单独停机、冻结积分或阻止地面准备/恢复。
接收端保留恢复一致的转换样本。`vertical_feedback_velocity_policy=diagnostic_only`、
`vertical_feedback_rate_warning`、差值/持续时间/样本数由状态接口输出。
NaN、无效数据、时间戳、估计器重置、真实位置残差、飞行包线、Kill、控制权保护保持原行为；
`VERTICAL_POSITION_RATE_INCONSISTENT` 仍是硬故障。
本轮没有修改推力、增益、1.5 m 高度、RC 档位或 YAML 配置。
开发电脑验证不代表已完成 Orange Pi 编译、DDS、实机服务切换或飞行验证。
