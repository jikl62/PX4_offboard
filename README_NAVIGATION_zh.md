# FC01 香橙派导航定高入口（2026-10-28）

本轮修复与验证记录见 [2026-11-03 起飞审查](docs/TAKEOFF_AUDIT_20261103.md)，通信取证见 [通信审查](docs/COMMUNICATIONS_AUDIT_20261103.md)。当前源码不会自动更新已部署的香橙派服务。

当前线束声明与 FC01 运行端点仍有冲突：用户报告现用为 **香橙派 UART2 ↔ FC01 USART6/RX6-TX6**，候选为 **香橙派 UART6 ↔ FC01 UART4/TELEM3**；现场 DDS 日志却显示 `/dev/ttyS3`，本地源码将其映射为 UART4。两条物理映射均暂不标记为已验证，候选保持未启用。见[双串口审计与只读工具](docs/FC01_DUAL_UART_20261105.md)；以下安装包不启用第二链路。

此包实现开机后台控制、遥控器独立解锁、CH6 导航触发和只读状态窗口。目标相对高度默认 **1.5 m**；20 Hz；起飞/定高任务总时限 **30 秒**，随后进入原生降落流程。默认运行配置只控制姿态、航向和相对高度。

视觉开发分支增加可选的视觉水平位置/速度外环、点目标和连续运动输入，默认关闭。临时接口、无硬件回放和合并步骤见 [视觉导航说明](docs/VISION_NAVIGATION.md)，本轮代码验证边界见 [验收记录](docs/VISION_VERIFICATION_20261031.md)。新相机接口与实机视觉飞行仍需分别验收。

## 一次安装

飞机保持未解锁、接地、静止。将下载的文件放到香橙派桌面，用普通 `orangepi` 用户运行：

```bash
FC01_NAVIGATION_INSTALL_DIR="$HOME/FC01_OrangePi_Navigation_20261103_takeoff" bash ~/Desktop/FC01_OrangePi_Navigation_20261103_takeoff.sh
```

安装会读取已有 Humble / `install_r632` 消息环境，单任务离线编译私有 C++ 控制器，用实际 rclcpp 参数解析检查完整 YAML，再被动观察新鲜的未解锁、接地、静止反馈。通过后停用已验证的旧 `fc01-standby.service`，启用新 `fc01-navigation.service`。保留原工程和旧入口。

不下载依赖、不重编 px4_msgs。需要香橙派已经有 `cmake`、`g++`、Humble 和原工程 SDK。首次构建耗时取决于机器；终端提示“正在离线编译”时继续等待。失败会在 `~/FC01_navigation_report_*.zip` 汇总诊断，先发这一个 ZIP 即可。

安装后当前终端自动显示状态；以后桌面登录会自动打开状态终端，也可点击桌面的“FC01 导航控制状态”。后台启动不依赖桌面登录。关闭状态窗口不会停止控制。

## 实际使用顺序

1. CH6 置 **Stabilized** 档位，CH5 置未解锁，CH7 Kill 保持正常，油门最低，其他杆居中；飞机接地静止。
2. 等待状态显示 **READY_WAIT_ARM**。这代表真实预发送和 PX4 信号条件已通过；普通 CONNECTED 不等于可以起飞。
3. 用 **CH5 手动解锁**，然后在 **8 秒内**将 **CH6 拨入 Offboard／导航位置**。本工程 `navigation_settings.json` 中的 `navigation_mode_slot` 为 **1**，对应此前飞控报告的 Offboard 档位。CH6 是遥控器物理通道号，和飞控的 `mode_slot` 是两回事。维持最低油门、其他杆居中。程序确认实际解锁和实际 Offboard 后，执行一次起飞定高。
4. 观察实际状态：`THRUST_RAMP（离地未确认）→ ASCEND（反馈确认离地）→ HEIGHT_HOLD（1.5 m 到高稳定）`。`navigation_allowed=true` 后才允许视觉运动；反馈确认仍需目视或录像核对。30 秒总任务时限结束会转入降落；新的接地证据满足后可自动上锁。
5. 再次执行前，接地上锁、CH5 OFF、CH6 回 Stabilized，等待完整重新待命。正常结束后在同一控制进程内重新准备，不再为下一轮任务退出并等待后台 30 秒重试；反馈持续有效时复用已有静止时间，再验证实际零推力预发送和新授权。进程异常退出仍保留有界重连等待。保持导航拨杆不会重复起飞。

必须先在 Stabilized 手动解锁，再拨 Offboard。程序不会发送自动 ARM 或主动进入 Offboard 的命令。开机时已经解锁、导航杆已保持、准备前就解锁、顺序错误或新鲜反馈不足，均不启动任务。

CH6 拨离导航会取消任务。程序观察实际模式变化；仍处于 armed Offboard 时沿用原生受控退出流程，不会主动抢回 Offboard。杆量变化、其他开关变化、Kill、失联和故障均保留原控制器的接管/故障策略。`land`/`stop` 会等待原生恢复结束，不能当成瞬时紧急断电。

## 管理

```bash
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh monitor
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh status
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh land
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh start
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh logs
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh report
```

`monitor` 只读；Ctrl+C 仅退出查看。`land` 与 `stop` 均请求原生优雅退出；`start` 启动等待下一次全新授权。`disable` 停止并停用导航自启。

回到旧的关闭动作门待命服务：

```bash
bash ~/FC01_OrangePi_Navigation_20261103_takeoff/navigation.sh rollback
```

回滚也必须先取得新鲜的未解锁接地证据。旧服务只待命，不会自动起飞。原安装目录和服务文件均保留。升级过程中管理员命令失败时，新旧服务不会被强杀；用 `report` 提供完整状态后再处理。

## 已知验证边界

本包的主机验证覆盖真实 C++ 状态机、最终发布边界、模拟定高/降落/上锁、消息乱序、失败注入，以及后台/安装生命周期；并不等于实机悬停验收。真正 ARM64/Humble 构建和 YAML 检查会在香橙派安装时执行。

之前实飞曾因 `VERTICAL_VELOCITY_INCONSISTENT` 退出；本包**保留原阈值和保护**，没有根据现有资料宣称修复垂直估计问题。候选 hover=0.475、起飞余量=0.040 原样保留，未作新实机标定。如果故障重现，状态会显示具体原因，并在受限日志里保留证据。

运行记录：`~/FC01_navigation/`。native 日志 4 MiB × 3，管理/安装日志各 1 MiB × 3；CSV 关闭。Agent 使用 `/dev/ttyS2` 115200，复用相同用户匹配进程；未知占用会阻止启动。窗口和日志不充当飞控租约。

目标和导航档位在 `navigation_settings.json`：

```json
{"target_height_m": 1.5, "navigation_mode_slot": 1}
```

`navigation_mode_slot` 必须为 1–6 的整数，需与飞控实际报告的 Offboard 档位一致。旧设置没有此字段时沿用基础配置的 `rc_navigation_mode_slot`；基础配置也未提供时保持旧缺省 6。更改设置后需在未解锁接地时重新执行 `navigation.sh install` 验证和重启。改变目标不会绕过高度、倾角、推力和估计器保护。

## 2026-10-31 源码修复

本次修复贯通导航档位配置；在手动解锁的独立反馈到达顺序不一致时，在原有证据等待时限内持续零推力准备输出，完整授权后才推进起飞。状态机与最终发布检查共同约束此等待。

后台重试保留最后反馈和接收年龄，状态窗口区分当前反馈与历史记录；IPC 关闭分别记录 `clean_eof`、`truncated_json`、`invalid_message`、`socket_error`，同时保留控制器退出码/信号。原生状态增加期望档位、状态反馈年龄、高度参考是否建立及高度不可用原因，便于和实际观测档位、保护原因对照。

源码修改后需在香橙派重新编译安装，运行中的旧 C++ 程序不会随源码自动更新。把修复后的工程放入新目录后，在该目录运行 `bash navigation.sh install`；安装沿用原有构建、YAML 检查和被动地面确认流程。使用自解压安装包更新时，应指定一个新的解压目录，原解压器会保留并拒绝覆盖内容不同的旧工程。

本机离线验证结果和部署说明见 [修复记录](docs/navigation_repair_20261031.md)。DDS 零回复问题及实际离地表现仍需香橙派与飞控的运行证据。
