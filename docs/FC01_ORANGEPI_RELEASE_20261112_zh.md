# FC01 20261112.1 香橙派源码安装包说明

本次交付文件为 `FC01_OrangePi_Navigation_20261112_release.sh`、同名 `.sh.sha256` 校验文件，以及本说明。默认解包目录是：

```text
~/FC01_OrangePi_Navigation_20261112_release
```

发行版本是 `20261112.1`。控制器中的 `FC01_RECOVERY_DIAGNOSTIC_V5_20261110` 是已有能力标记，继续保留；它不表示仍在使用旧版源码。具体源码身份以包内 `FC01_RELEASE.json` 的 `source_commit` 和逐文件 SHA256 为准，服务版本以实际状态中的 `entry_version` 为准。

这是供目标板原生构建的源码包，不包含预先认证的 ARM64 可执行程序，也不包含操作系统、ROS 2、PX4 固件或 Agent 安装包。新控制器、参数检查器、五种自定义消息及其类型支持，在本次解包目录内构建。旧 SDK 只提供 ROS/PX4 消息等依赖，新控制器和飞行配置使用本包内容。

## 1. 先分清三个动作

| 动作 | 正式入口 | 实际效果 |
| --- | --- | --- |
| 仅解包 | 安装包 `--install-only`（也支持 `--extract-only`） | 校验并放置源码，不构建、不切换服务 |
| 离线准备 | `navigation_deploy.sh offline` | 本机 ARM64 构建、检查配置、生成证据；不启动控制器、Agent 或服务 |
| 在线启用 | `navigation_deploy.sh activate --confirm-flight-test` | 检查离线证据和现场条件，切换并启动服务 |

直接执行安装包而不带参数，会在解包后执行 `offline`。`activate` 会启动服务；如果希望先安装服务定义、随后由自己手动启动，使用第 4 节的 `configure` 路线。

本包保留人工 RC 解锁及原有无视觉 1.5 m 定高路线。程序解锁保持关闭，视觉导航保持 `visual_navigation_enabled=false`。启动服务并不等于获得起飞许可；RC、飞控数据、安全锁存和原生状态机仍决定是否允许进入任务。本文不修改 PX4 参数、推力或控制器保护条件。

## 2. 香橙派上需要已经具备什么

- 实际运行 ARM64 Linux，`uname -m` 为 `aarch64` 或 `arm64`；使用普通 `orangepi` 用户运行入口，不使用 `sudo bash` 执行整套安装器。
- `/opt/ros/humble/setup.bash`、系统 `/usr/bin/python3`、PyYAML、CMake、GCC/G++、Make、Binutils，以及 ROS 2 Humble 的 `ament_cmake`、`rclcpp`、`rosidl_default_generators`、`builtin_interfaces`。视觉工具另需 `rclpy`、`rcl_interfaces`、`nav_msgs`。
- 已有 SDK 下的 `install_r632/local_setup.bash`，其中 `px4_msgs` 必须与实际 PX4 固件接口匹配。仅仅编译成功不能证明实机消息版本一致。
- 在线阶段需要 `systemd`、`sudo`、`fuser`，以及当前用户对 `/dev/ttyS2` 的读写权限。
- 在线配置/激活需要已经存在的、同用户、独占 `/dev/ttyS2`、波特率 115200 的兼容 MicroXRCEAgent。导航运行环境还检查 `~/fc01_agent_build_offline/MicroXRCEAgent` 可执行文件。

源码包不会联网自动安装这些依赖。缺少依赖会明确报错，先保存日志，再按板上现有系统环境补齐。不要删除旧 SDK、旧发行目录、`~/FC01_navigation` 或回滚备份。

## 3. 校验、解包和离线 ARM64 构建

下面假设安装包和校验文件放在香橙派的 `~/Desktop`。如果实际目录不同，只调整第一行。

```bash
cd "$HOME/Desktop"
sha256sum -c FC01_OrangePi_Navigation_20261112_release.sh.sha256
bash ./FC01_OrangePi_Navigation_20261112_release.sh --install-only
```

SHA256 应显示 `OK`，解包应显示 `FC01_EXTRACT_ONLY_COMPLETE`。已有同名目录时，解包器只接受与本包逐字节一致的文件，不覆盖用户改过的源码。不要用删除旧目录的方法绕过报错；先核对目录和日志。

接着指定本包目录及已有 SDK。以下 SDK 路径是现有默认值；如果板上的真实 SDK 位于其他目录，必须改成那个绝对路径。不要把新发行目录误填成 SDK。

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
test -f "$FC01_PROJECT_ROOT/install_r632/local_setup.bash"
bash "$FC01_RELEASE_DIR/navigation_deploy.sh" offline
```

以下操作沿用这个终端中的 `FC01_PROJECT_ROOT`。打开新终端后，如果 SDK 不在默认目录，需要重新执行上面的 `export FC01_PROJECT_ROOT=...`，或在本包外部生成的 `deployment-config.json` 中按既有机制指定绝对 `sdk_root`。

离线入口会自行加载 Humble 和所选 SDK。它使用单任务构建，首次构建及依赖内容摘要可能需要较长时间。不要因为暂时没有新输出而同时启动第二个安装器。

成功时应出现 `FC01_ARM64_FRESH_BUILD_VERIFIED` 或严格匹配证据后的 `FC01_ARM64_BUILD_REUSED`，最后出现 `FC01_OFFLINE_READY`。该目录内会生成：

- `ARM64_BUILD_EVIDENCE.json`：源码、依赖、工具链、AArch64 ELF、安装树和动态库证据。
- `OFFLINE_DEPLOYMENT.json`：本轮离线检查成功记录。
- `deployment/flight-test-runtime.yaml`：本包局部的待检查配置，不是正在运行的配置。
- `install`：指向本包 `.build/arm64/` 内构建结果的链接。

离线成功只证明本机构建和配置检查通过，没有运行生产控制器，没有检验 PX4 现场状态。构建后保留整个发行目录，不移动目录或单独复制 `install`；证据绑定真实路径和依赖内容。源码、SDK 或编译器变化后，重新执行 `offline`。

当前设置可只读查看：

```bash
cat "$FC01_RELEASE_DIR/navigation_settings.json"
```

本次设置应为 `target_height_m: 1.5`、`navigation_mode_slot: 1`。后者是飞控模式档位编号，不是 RC 通道号，也不能据此直接推断遥控器开关的位置。不要在完整性校验后直接改包内文件再继续激活；修改会使发布摘要失效。

## 4. 先配置、之后自己手动启动

这一节是分步操作路线。在线操作及首次启动安排在拆桨、未解锁、接地静止、RC 复位的条件下进行；不是动力试飞流程。

先只读查看现有托管服务，`--no-pager` 可以避免进入 `less` 的 `(END)` 窗口：

```bash
systemctl --no-pager show fc01-navigation.service fc01-standby.service \
  -p Id -p ActiveState -p MainPID -p WorkingDirectory -p UnitFileState
```

`configure` 要求两个托管服务均已停止，并且没有遗留控制器。它发现活动服务会拒绝，自己不会隐式停止旧控制器。只对现场确实存在、且已确认可以安全停止的托管服务执行对应命令：

```bash
sudo systemctl stop fc01-navigation.service
```

如果旧 `fc01-standby.service` 也存在并运行，单独停止它：

```bash
sudo systemctl stop fc01-standby.service
```

停止会请求原生控制器正常交权/退出，可能持续等待。不要使用 `kill -9`、`pkill` 或 `systemctl kill` 跳过等待。若旧 standby 仍有开机启动入口，`configure` 会提示；现场确认后再单独执行：

```bash
sudo systemctl disable fc01-standby.service
```

停止旧服务后，再核对服务已停止。复用的既有 Agent 通常会保留；旧服务如果拥有自己创建的 Agent，也可能在安全退出时关闭它。此时 `configure` 会因缺少可复用 Agent 而拒绝，需先按既有传输管理方式恢复 Agent，不要重复占用串口。

如果原 Agent 确实已退出，且 `fuser /dev/ttyS2` 确认串口没有占用者，可以在另一个普通用户终端前台启动现有传输程序，并保持该终端打开；已有 Agent 时跳过这一步。该命令只启动 DDS 传输，不启动控制器：

```bash
"$HOME/fc01_agent_build_offline/MicroXRCEAgent" serial \
  -D /dev/ttyS2 -b 115200 -v 0
```

若串口仍被占用或 Agent 启动报错，停止本次配置并保存输出，不另外启动第二个 Agent。

然后安装服务定义，但保持停止：

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
bash "$FC01_RELEASE_DIR/navigation_restore.sh" configure \
  --confirm-flight-test --boot-policy preserve
```

应看到 `FC01_NEW_SERVICE_CONFIGURED`、`FC01_EXISTING_AGENT_RETAINED`、`FC01_SERVICE_NOT_STARTED` 和 `rollback=...`。保存这个完整备份路径。`preserve` 表示保留原开机策略；原来已启用开机启动的服务仍会保持启用，不代表已禁用开机启动。若有 drop-in、未知控制器、旧开机入口或地面反馈不合格，保留现场并处理具体原因。

`configure` 是既有分步入口：它检查本包二进制能力标记、实际配置及被动地面证据，但不代替 `navigation_deploy.sh activate` 的完整构建证据/离线令牌/启动后状态事务验收。务必先完成第 3 节，不修改其后生成的源码和产物。需要完整事务自动验收时，使用第 5 节；不要把两条路线连续执行当成必需步骤。

确认自己准备开始无动力运行时，再明确手动启动：

```bash
sudo systemctl start fc01-navigation.service
systemctl --no-pager show fc01-navigation.service \
  -p ActiveState -p MainPID -p WorkingDirectory
bash "$FC01_RELEASE_DIR/navigation.sh" status
```

`WorkingDirectory` 应是新发行目录。`active` 只说明 systemd 服务进程存在，还应检查 `status` 的真实飞控反馈、控制器身份、当前阶段和故障原因，不能把它直接当作飞行验收成功。

## 5. 可选：完整在线激活事务

如果选择由部署器完成旧托管服务停止、配置切换、新服务启动和启动后验收，使用这一条路线。它需要已完成第 3 节，并会实际启动服务：

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
bash "$FC01_RELEASE_DIR/navigation_deploy.sh" activate \
  --confirm-flight-test --boot-policy preserve
```

部署器会重新核对构建证据、控制器来源、既有 Agent 和新的 RC/地面证据，并保存回滚记录。成功后有 `FC01_DEPLOY_COMPLETE` 和 `FC01_ROLLBACK_PATH=...`。失败应查看对应日志及最终服务状态，不要无限重试，不要手工删除锁文件，不要强杀控制器。

这一在线事务成功也不是飞行性能结论；它仍是在地面、未解锁条件下验收服务身份和反馈。

## 6. 状态、日志与测试后的报告

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
bash "$FC01_RELEASE_DIR/navigation.sh" status
bash "$FC01_RELEASE_DIR/navigation.sh" logs
bash "$FC01_RELEASE_DIR/navigation.sh" report
```

`report` 会打印类似 `~/FC01_navigation_report_时间.zip` 的路径。测试后把该 ZIP 和当时的现场方式、发生时间、遥控器操作顺序一起提供，便于区分旧状态、现场数据失效和代码缺陷。状态/报告命令不请求起飞；关闭状态窗口不停止控制服务。

部署日志位于 `~/FC01_navigation/deploy-logs/FC01_OrangePi_Navigation_20261112_release/`，入口还会打印具体日志路径。离线构建日志位于 `~/FC01_navigation/offline-build-*.log`，运行日志和状态在 `~/FC01_navigation/`。不要只保留终端中最后一行。

仅用于拆桨地面测试的显式停止入口为：

```bash
bash "$FC01_RELEASE_DIR/navigation.sh" stop
```

它请求托管服务正常停止；原生退出条件仍生效。`land` 也沿用此安全退出映射，不是 Python 绕过原生状态机直接发送 PX4 降落命令。

## 7. 回滚到已保存的服务定义

使用第 4 或第 5 节实际打印的备份路径，替换下面示例。不要照抄示例时间戳，也不要猜选“最新目录”。

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
FC01_ROLLBACK_DIR="$HOME/FC01_navigation/service-backups/实际打印的备份目录"
bash "$FC01_RELEASE_DIR/navigation_restore.sh" rollback \
  --backup "$FC01_ROLLBACK_DIR"
```

回滚入口会在有需要时请求当前托管服务安全退出，并重新取得地面证据，校验备份和现有文件后恢复。成功提示 `FC01_SERVICE_ROLLED_BACK_NOT_STARTED`：旧服务定义/开机策略已恢复，但旧服务不会自动重启。回滚后先只读核对 `WorkingDirectory` 和状态，再由人决定是否启动。保留新旧发行目录及备份，直到确认恢复结果。

## 8. 当前验证边界

包内包含无视觉起飞控制源码、ROS 2 消息、视觉状态适配和目标速度只读诊断。Odometry 是飞机实际状态估计；VelocityCommand 是制导目标，两者不能互相替代。消息生成不授予视觉控制权限，本包不会自动启动视觉节点。

发行打包校验、x86 ROS 构建或模拟消息测试不能代替 Orange Pi Zero 3W 的 ARM64 原生构建、CPU/内存/磁盘负载验证、串口/DDS 时序、实际 PX4 反馈和实机试验。已有安全候选补丁保持隔离；制作新包并不表示这些候选已纳入控制器。先完成板上的无动力构建与服务身份验收，再单独安排后续硬件验证。
