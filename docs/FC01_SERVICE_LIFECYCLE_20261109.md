# FC01 服务生命周期与独立升级说明（20261109）

20261110 地面参考与发布周期修复版已更新安装包、独立目录和构建标识；升级本版请使用 [20261110 针对性修复说明](FC01_REFERENCE_PUBLICATION_20261110.md)。下文保留 20261109 生命周期策略与历史证据。

本版恢复持久服务的开机待命能力，并将一次 RC 飞行任务的结束与导航服务的退出分开。服务可以持续运行；正常任务完成后，以及受支持的故障已经解除并重新取得安全地面证据后，控制器重新进入下一轮准备。这里的恢复不授权自动解锁、自动起飞或空中重启任务。

本轮未连接香橙派、未部署、未执行实际重启或动力试验。本文提供可在香橙派执行的独立升级方法；新 ARM64 产物、重启自启及飞后现场行为，必须由对应实机证据确认。既往 ARM64/DDS 成功记录不能替代本版的 ARM64 构建证据。

## 版本、配置与交付边界

| 项目 | 本版要求 |
| --- | --- |
| 源码安装包 | `FC01_OrangePi_Navigation_20261109_lifecycle.sh` 与同名 `.sha256` 文件 |
| 独立目录 | `/home/orangepi/FC01_OrangePi_Navigation_20261109_lifecycle` |
| 既有 ROS/PX4 SDK | `/home/orangepi/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE` |
| Python 入口版本 | `20261109.1` |
| 原生程序标识 | `FC01_SAFE_LIFECYCLE_V1_20261109` |
| R2 安全事件策略 | `R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108` |
| 模式源文件 | 新目录中的 `native/config/diagnostic.yaml`、`native/config/flight_test.yaml` |
| 实际运行配置 | `/home/orangepi/FC01_navigation/runtime.yaml`；启动日志同时记录源配置路径 |

安装包仅含源码与配置，不含 x86 或历史 ARM64 控制器。源码提交由包内 `FC01_RELEASE.json` 记录；香橙派全新构建后，`ARM64_BUILD_EVIDENCE.json` 记录提交、二进制 SHA256、AArch64 ELF、ROS 2 Humble 与 PX4 消息依赖。包 SHA256 以随包交付的校验文件为准。

本轮不改推力：hover `0.50`、reserve `0.04`、min `0.12`、max `0.80`、slew `0.40/s`；目标相对高度 `1.5 m`。视觉关闭，保留无视觉 RC 人工解锁路线及视觉接口。`flight_enabled=false` 仍是此姿态定高配置的预期值；正式 Flight Test 入口显式生成 `flight_actions_enabled=true`、`takeoff_enabled=true`、`arm_enabled=false` 的运行配置，并保留 RC、安全反馈和状态确认守门。Diagnostic 动作门关闭。

## 服务与任务各自如何结束

| 情况 | 新行为与限制 |
| --- | --- |
| 正常任务结束 | 确认上锁、接地、控制权已释放后，服务保留；满足新一轮 RC 地面准备条件才重新准备 |
| 可恢复飞行故障 | 原保护继续执行；只有故障解除、持续健康多帧地面反馈和新的安全 RC 周期全部满足后，才允许重新准备 |
| 完整性、时间回退、未知故障 | 保留锁存，不承诺自动清除；必须查明原因，服务 active 也不代表能够起飞 |
| Agent 暂时不可用 | Supervisor 保留原生控制器，恢复或复用 Agent；原生程序处理反馈丢失，不因通信恢复直接重开飞行任务 |
| 原生进程异常退出 | 记录退出码/信号和原因，受控退避后再尝试；替代进程从新反馈重新建立准备证据 |
| Supervisor 异常退出 | 原生程序通过父进程死亡信号及周期父 PID 检查进入既有安全 shutdown；重启 Supervisor 等待旧控制器退出，期间只恢复 Agent 传输 |
| 用户明确停止服务 | 请求原生安全退出并等待，不因 `Restart=on-failure` 重新拉起；不强杀仍在等待安全退出的进程 |
| 香橙派重新上电 | 已 enable 的正式服务自动运行，从新鲜反馈开始准备；历史状态不作为当前飞机证据 |

正常完成和可恢复故障经安全入口重新建立任务临时状态与起飞参考；进程重建后，源时钟水位与反馈也必须从新样本重新建立。已提交 Disarm 的 ACK/线上命令历史与独立物理终态仍按原确认逻辑处理，不把清状态当作完成上锁。历史故障与每次原生退出保留在日志中。

`VERTICAL_VELOCITY_INCONSISTENT` 的飞行根因没有在本轮解决。本轮只允许其在受支持的安全恢复分支中，经上锁、接地、健康垂直反馈窗口、多帧推进以及地面 dwell 后重新准备；持续异常、Kill、坏四元数或同轮坏后好事件仍阻断恢复。它曾经发生过，不代表 systemd 停止就是由它引起。必须分开查看原生故障、原生退出和服务退出记录。

## systemd 策略

正式 unit 由 `navigation_install.py` 生成，部署到 `/etc/systemd/system/fc01-navigation.service`。以下是生命周期相关部分；完整 `ExecStart` 和 SDK 路径由本次独立目录生成：

```ini
[Unit]
After=local-fs.target
StartLimitIntervalSec=120s
StartLimitBurst=5

[Service]
Type=simple
Restart=on-failure
RestartSec=30s
KillMode=process
KillSignal=SIGTERM
SendSIGKILL=no
TimeoutStopSec=infinity

[Install]
WantedBy=multi-user.target
```

`Restart=on-failure` 仅处理 Supervisor 异常；普通任务完成不会退出 Supervisor。内部原生异常和 Agent 恢复使用受控重试。`KillMode=process` 配合原生父进程死亡处理，给既有安全 shutdown 留出时间；新 Supervisor 不越过单实例锁，不与仍在退出的旧控制器并发输出。等待期间可以保留或恢复 Agent，避免切断旧控制器的安全退出通信。

`configure --boot-policy enable` 只安装 unit 并 enable，不执行 start。首次升级由操作者人工 start 服务；以后系统启动自动进入准备流程。`enable` 不表示动作门已满足，也不表示当前允许起飞。显式 `systemctl stop` 后，当前启动周期保持停止；enable 的服务会在下次系统启动再次运行。

## U 盘传输与全新 ARM64 构建

以下命令以香橙派普通用户 `orangepi` 执行，不以 root 运行构建或整个安装脚本。先在无动力、上锁接地条件下操作。构建本身不需要启动控制器、停止旧服务或改变 Agent。

将最终 `.sh` 和 `.sh.sha256` 两个文件复制到 U 盘。在香橙派终端输入实际 U 盘目录：

```bash
read -r -p 'U盘中安装包所在目录: ' fc01_usb_dir
cd -- "$fc01_usb_dir"
sha256sum -c FC01_OrangePi_Navigation_20261109_lifecycle.sh.sha256
mkdir -p "$HOME/FC01_release_packages/20261109_lifecycle"
cp -n -- FC01_OrangePi_Navigation_20261109_lifecycle.sh FC01_OrangePi_Navigation_20261109_lifecycle.sh.sha256 "$HOME/FC01_release_packages/20261109_lifecycle/"
cd "$HOME/FC01_release_packages/20261109_lifecycle"
sha256sum -c FC01_OrangePi_Navigation_20261109_lifecycle.sh.sha256
```

两次校验必须成功。若目标安装目录已存在，停止此步骤，保留其内容并确认来源；不要删除、覆盖或将旧目录改名成新版。以下块使用全新目录，解包后执行强制新构建入口：

```bash
set -euo pipefail
fc01_release_dir="$HOME/FC01_OrangePi_Navigation_20261109_lifecycle"
test ! -e "$fc01_release_dir"
test ! -L "$fc01_release_dir"
export FC01_NAVIGATION_INSTALL_DIR="$fc01_release_dir"
bash "$HOME/FC01_release_packages/20261109_lifecycle/FC01_OrangePi_Navigation_20261109_lifecycle.sh" --install-only
cd "$fc01_release_dir"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
bash navigation_build_fresh.sh 2>&1 | tee "$HOME/FC01_release_packages/20261109_lifecycle/arm64-fresh-build.log"
```

该入口拒绝非 ARM64 主机、已有 `install`/`.build`/构建完成记录，重新校验包内源码清单，在 `.build/arm64-final` 编译到本目录 `install`。如果中途失败，不使用普通安装器的旧缓存跳过构建；保留错误日志，查明失败原因后另作明确的新目录构建安排。不要把旧 `fc01_system_node` 拷入新目录。

成功标志是 `FC01_ARM64_FRESH_BUILD_VERIFIED source_commit=...`、两个二进制 SHA256 和 `FC01_ARM64_BUILD_EVIDENCE=...`。然后核对证据和实物：

```bash
cd "$HOME/FC01_OrangePi_Navigation_20261109_lifecycle"
cat FC01_RELEASE.json
cat ARM64_BUILD_EVIDENCE.json
file install/lib/jikl_fc01_system/fc01_system_node
sha256sum install/lib/jikl_fc01_system/fc01_system_node install/lib/jikl_fc01_system/fc01_config_check
```

必须是 AArch64，摘要与证据一致。构建脚本已经检查动态库无 `not found`、PX4 消息依赖来自所选 SDK、ROS 2 Humble 以及新版原生标识。SDK 仅提供依赖，不作为二进制或运行推力配置来源。

## 停旧入口、配置持久服务并首次人工启动

先确认现场已上锁接地、控制权释放；本节不是空中更新方法。保留既有 MicroXRCEAgent，不执行 Agent 的 pkill、重复启动或串口抢占。

```bash
sudo systemctl stop fc01-navigation.service
systemctl show fc01-navigation.service -p ActiveState -p SubState -p MainPID
pgrep -af '[/]fc01_system_node([[:space:]]|$)' || true
systemctl show fc01-standby.service -p LoadState -p ActiveState -p UnitFileState -p MainPID
pgrep -af '[M]icroXRCEAgent' || true
```

等待旧服务停止且没有 `fc01_system_node`。如果停止仍在等待，不强杀、不开始配置第二个控制器。若历史 `fc01-standby.service` 存在且运行或启用，还应在相同安全地面条件下停止并禁用这个旧启动入口：

```bash
sudo systemctl stop fc01-standby.service
sudo systemctl disable fc01-standby.service
pgrep -af '[/]fc01_system_node([[:space:]]|$)' || true
```

不存在旧 standby unit 时跳过这两条命令。不要 disable 正要更新的 `fc01-navigation.service`，这样配置工具可以备份其原始持久状态。配置器会再次检查旧服务、未知 drop-in、单实例、既有 Agent 和新鲜被动地面证据；任何拒绝都应保留现场并阅读原因。

```bash
set -euo pipefail
cd "$HOME/FC01_OrangePi_Navigation_20261109_lifecycle"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
bash navigation_restore.sh configure --confirm-flight-test --boot-policy enable 2>&1 | tee "$HOME/FC01_release_packages/20261109_lifecycle/configure.log"
```

必须同时看到：

```text
FC01_NEW_SERVICE_CONFIGURED controller=/home/orangepi/FC01_OrangePi_Navigation_20261109_lifecycle/install/lib/jikl_fc01_system/fc01_system_node
FC01_EXISTING_AGENT_RETAINED pid=...
FC01_SERVICE_NOT_STARTED boot_enabled=enabled boot_policy=enable original_enabled_state=... rollback=/home/orangepi/FC01_navigation/service-backups/...
```

立即将最后一行 `rollback=` 后的完整路径保存到现场记录，作为本次更新唯一回滚入口。它包含原 unit 字节、原 `UnitFileState` 和摘要；不要凭日期猜一个备份。确认配置结果后，首次由操作者启动服务：

```bash
systemctl is-enabled fc01-navigation.service
systemctl cat fc01-navigation.service
sudo systemctl start fc01-navigation.service
systemctl show fc01-navigation.service -p ActiveState -p SubState -p MainPID -p Result -p NRestarts
python3 "$HOME/FC01_OrangePi_Navigation_20261109_lifecycle/navigation.py" status
journalctl -u fc01-navigation.service -n 100 --no-pager
```

unit 的 `WorkingDirectory` 和 `ExecStart` 必须使用 20261109 独立目录。服务启动只进入既有准备路线，不自动发送解锁；不得根据 `active (running)` 推定飞行条件已经满足。

核对实际运行文件、单实例和摘要：

```bash
pgrep -af '[/]fc01_system_node([[:space:]]|$)'
fc01_native_pid=$(pgrep -f '[/]fc01_system_node([[:space:]]|$)')
test -n "$fc01_native_pid"
readlink -f "/proc/$fc01_native_pid/exe"
sha256sum "/proc/$fc01_native_pid/exe"
pgrep -af '[M]icroXRCEAgent' || true
```

必须只有一个 native PID，文件路径是本版 `install/lib/jikl_fc01_system/fc01_system_node`，摘要匹配 `ARM64_BUILD_EVIDENCE.json`。多 PID、未知来源或摘要不一致均停止推进。

启动 JSON 的 `controller_build=FC01_SAFE_LIFECYCLE_V1_20261109`、`entry_version=20261109.1`、`runtime_mode=flight-test`、实际 `config_path` 和新目录内 `config_source_path` 都要匹配。确认 `aa_hover_thrust=0.50`、`aa_takeoff_reserve_thrust=0.04`、`aa_thrust_max=0.80`、`rc_target_height=1.5`、`flight_enabled=false`、`arm_enabled=false`、`visual_navigation_enabled=false`，动作门来自显式 Flight Test profile。日志仍显示旧标识或旧参数时，不继续使用该实例。

## 残留 IPC、重启与飞后状态检查

每次 native 启动使用新的私有 IPC 子目录。旧 socket 不作为当前健康证据，也不为了恢复而批量删除；仍可能属于活跃进程的 socket 必须保留。`SOCKET_PRESENT_LIVENESS_UNKNOWN` 表示证据不足，不能据此强删、强杀或放行第二实例。只有本轮创建且已确认原生退出的 IPC 才由对应 Supervisor 清理。

`status.json` 同时绑定本次 boot ID、Supervisor PID、更新时间与反馈新鲜度。旧 boot 或旧进程快照只能显示历史；控制器源时钟水位重新由新反馈建立，不能沿用上轮记录放行动作。

正常飞行结束或受支持的故障恢复后，关键日志为：

```json
{"kind":"rc_safe_reprepare","reason":"NEW_DISARMED_GROUND_AND_CONTROLS_RESET","completed_attempt":1,"previous_reason":"..."}
```

这说明上一轮满足重新准备入口，不是下一轮起飞许可。还应看到本轮新鲜反馈中已上锁、接地/静止、退出 Offboard、RC 安全、当前故障解除，以及重新进入 `RC_PRESTREAM` / `RC_READY_WAIT_ARM`（状态视图可显示 `PRESTREAM` / `READY_WAIT_ARM`）。RC 必须出现新的安全准备周期；仅服务 active、历史高度回到零或旧任务状态被清空，都不能证明可以继续动力试验。

本文件不提供真实动力起飞动作序列。现场操作者依既有验收规程决定是否进行下一次任务；保留人工 RC 解锁、Kill、接管和 PX4 Failsafe。若仍 armed、接地未知、反馈过期、RC 不安全、垂直反馈持续异常或完整性锁存未解除，则下一任务继续被阻止。

在拆桨地面条件下，由现场人员完成系统重启后，使用前述 `is-enabled`、`show`、`status` 和日志命令检查自启，并核对 boot ID、本次 PID、新鲜反馈及单实例。不要把“已 enable”写成“本版实机重启通过”。

日志采集命令：

```bash
cd "$HOME/FC01_OrangePi_Navigation_20261109_lifecycle"
python3 navigation.py report
journalctl -u fc01-navigation.service -b --no-pager > "$HOME/FC01_release_packages/20261109_lifecycle/service-current-boot.log"
systemctl show fc01-navigation.service -p ActiveState -p SubState -p Result -p NRestarts -p ExecMainCode -p ExecMainStatus > "$HOME/FC01_release_packages/20261109_lifecycle/service-state.txt"
```

报告 zip 包含当前/历史状态、运行配置、控制器与 Supervisor 日志及 journal。另保留包校验文件、`FC01_RELEASE.json`、`ARM64_BUILD_EVIDENCE.json`、构建/configure 日志和本次 rollback 路径。

## 停止与回滚

在现场确认安全地面后请求正常停止；命令可能等待原生安全退出，不使用 SIGKILL：

```bash
sudo systemctl stop fc01-navigation.service
systemctl show fc01-navigation.service -p ActiveState -p SubState -p MainPID
pgrep -af '[/]fc01_system_node([[:space:]]|$)' || true
```

仅当服务停止且无控制器后，使用本次 configure 记录的 backup 完整路径回滚。保留既有 Agent，工具还会重新取得被动地面证据：

```bash
set -euo pipefail
cd "$HOME/FC01_OrangePi_Navigation_20261109_lifecycle"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
read -r -p '本次configure输出的rollback完整路径: ' fc01_rollback_backup
bash navigation_restore.sh rollback --backup "$fc01_rollback_backup"
systemctl cat fc01-navigation.service
systemctl is-enabled fc01-navigation.service || true
systemctl show fc01-navigation.service -p ActiveState -p SubState -p MainPID
```

成功标志 `FC01_SERVICE_ROLLED_BACK_NOT_STARTED`。回滚恢复原 unit 及原持久启用状态，不自动 start。原目录一直保留，未改名、未覆盖、未链接冒充新版；若当前 unit 自 configure 后被另外修改，工具拒绝覆盖。现场审核恢复后的 `ExecStart`、参数和实际安全证据后，才由操作者另行决定是否启动旧服务。

## 本轮软件证据与待现场验证项目

本轮已通过 100 项受影响 Python 回归、12 项受影响 C++ 测试及父进程死亡真实 fork 测试；使用全新目录完成 ROS 2 Humble x86_64 原生节点编译链接。这是软件回归和 x86 构建证据，没有本轮新的 ARM64 编译或硬件重启证据。

针对性回归覆盖正常任务后重新准备、真实 VVI 故障在 armed 时阻止恢复、持续异常/Kill 阻断、健康多帧地面 dwell 后恢复、同轮坏四元数后正常样本不能覆盖异常、重新人工 RC 周期、冷启动 armed/未知状态不输出，以及完整性故障继续锁存。R2 接收/离散安全事件策略保持。

现场仍须验证本版全新 ARM64 构建与摘要、正式 unit 自启、单控制器与 Agent 重连、实际 RC 接管/Kill、Agent 断开时原生安全退出与 PX4 Failsafe、正常落地后的新鲜安全周期及对应日志。上一轮 VVI 的实际传感器/时间/估计根因仍待分析。本轮不能据此声称已经具备直接连续动力飞行的实机证据；若任一当前保护仍触发，应保留记录并解决原因。
