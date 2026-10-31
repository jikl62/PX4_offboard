# FC01 香橙派开机待命

在香橙派上以 orangepi 普通用户执行一次：

```bash
bash ~/Desktop/FC01_OrangePi_Autostart_20261027.sh
```

按系统提示输入一次 sudo 密码。脚本离线展开到 `~/FC01_OrangePi_Autostart_20261027`，自动检查已安装的 ROS Humble、px4_msgs、R632 ARM64 控制器和关闭动作门的配置，安装并启用 `fc01-standby.service`，立即启动。以后香橙派上电即可后台待命，不需要登录桌面或打开终端。没有下载或重编译步骤，不修改原工程与飞控参数。

这一步完成**开机后台连接与待命**。所有动作门关闭，不会自动进入 Offboard、解锁、起飞；当前 CH6 拨杆也不会触发任务。你选择的 A 流程（CH6 导航请求→就绪→CH5 手动解锁→自主定高）将在下一阶段适配原生状态机；旧交互入口的 `takeoff` 仍会由程序请求解锁，不能视为 A 流程已经实现。

## 查看与管理

```bash
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh status
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh logs
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh report
```

`enabled` 表示已设置开机启动，`active` 表示后台服务运行。后台 `CONNECTED` 表示刚收到有效期内的飞控状态/本地位置，不代表起飞许可；缺少的反馈条件另行显示。`WAITING` 会给出缺少设备、权限、已有控制器或反馈的原因，每 30 秒重试一次，不会高速反复重启。若香橙派先上电，飞控稍后上电，服务会等待连接。

`report` 生成一个 ZIP，显示完整路径；需要排查时只需提供这个文件。状态和轮转日志保存在 `~/FC01_autostart/`：原生控制器 4 MiB×3，主管 1 MiB×3；后台通过 YAML 空字符串禁用 CSV，同时关闭 ROS 额外的文件日志，将控制器输出集中写入轮转日志。状态快照每 5 秒覆盖更新，等待依赖时也持续更新。自建 Agent 静默运行；复用 Agent 的外部日志不由本包管理。

```bash
# 停止当前后台待命，保留下次开机自启
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh stop
# 恢复后台待命
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh start
# 停止并取消开机自启
bash ~/FC01_OrangePi_Autostart_20261027/autostart.sh disable
```

停用会等待原生控制器优雅退出，不强杀。复用的 115200 Agent 始终保留；自建 Agent 只有在控制器退出且有新鲜未解锁接地证据时才停止，否则保留，下一次启动可复用。systemd 仅向待命主管发送停止信号，由主管管理原生控制器；这样保留 Agent 后仍可重新启动。若主管异常退出，尚在运行的原生控制器会阻止新控制器启动，状态会给出等待原因；排他锁不会被子进程继承。若原生退出等待恢复，管理命令也会等待，不应通过强杀跳过。

旧入口和待命服务共享排他锁，不能同时运行。需要回到旧入口时先停止后台服务，再启动旧入口；完成旧会话后可恢复后台待命。重复安装相同版本不会重启已在运行的服务，也不会覆盖不同文件、原入口或不属于本包的同名服务。

当前固定 `/dev/ttyS2 @115200`、ROS domain 0，使用候选工程的 `install_r632`；自定义工程位置在首次安装前设置 `FC01_PROJECT_ROOT`。`--install-only` 仅展开文件，不安装服务；一般使用默认无参数安装即可。

本包交付前进行主机上的离线解包、系统服务语法、套接字生命周期、关闭动作门及日志轮转验证。未通过远程连接在你的香橙派上安装或验证开机。此前实飞的 `VERTICAL_VELOCITY_INCONSISTENT` 尚未解决；本包保留该保护。
