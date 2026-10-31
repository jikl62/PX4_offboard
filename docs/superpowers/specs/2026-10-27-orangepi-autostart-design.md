# 香橙派后台待命自启

用户选择 A：CH5 负责遥控解锁，CH6 模式位置请求导航，CH7 保留急停。本次明确先交付开机自启；后续才实现拨杆导航任务。用户已要求直接开始、减少确认，本次按已有授权执行。

## Approach

使用 systemd 系统服务，以 orangepi 普通用户启动独立后台待命入口，沿用现有 R632 原生控制器和配置解析。直接将交互 --flight 放进 systemd 会因为无 TTY 退出，也会把尚未适配的自动解锁路径带入后台。只启动 Agent 无法证明当前控制代码可随系统启动。因此选择原生控制器的关闭动作门待命模式。

## Scope and behavior

- 一个离线脚本，展开至独立目录，安装并启用 fc01-standby.service，立即后台启动，不需要桌面登录、网络或持续终端。
- 自动加载 /opt/ros/humble/setup.bash 和候选工程 install_r632/local_setup.bash；固定 domain 0、/dev/ttyS2、115200。
- 生成独立 runtime.yaml，八个 ACTION_FLAGS 全部 false；保留高度反馈保护、原工程和 PX4 参数。后台只发 STATUS，不领取交互飞行 lease，不准备 Offboard、解锁或起飞。
- 复用同用户、同端口、115200 Agent；未知占用和已有控制器保留。复用 Agent 不停止。原生进程启动与运行时持有和旧入口相同的排他锁；主管异常退出后的子进程保护依赖进程扫描，子进程不继承锁。
- 启动依赖未就绪时 30 秒间隔等待并显示原因；不会高速循环重建传输。原生进程意外退出后回到关闭门待命，最多每 30 秒尝试一次。
- SIGTERM 请求原生优雅退出并等待，不强杀。自建 Agent 仅在已有 finish_agent 判定的新鲜未解锁接地证据下停止，否则保留。自建 Agent 使用静默输出到 /dev/null，保留时不会因日志管道断开而退出。
- 原生 stdout 使用 4 MiB、两份备份的轮转日志；待命主管日志 1 MiB、两份备份。YAML 空字符串禁用原生 CSV，关闭 ROS external-library 文件日志。状态快照原子替换，每 5 秒更新一次（包括依赖等待期间）。使用固定待命数据目录，不产生每次重试的新会话大文件。
- systemd KillMode=process、SendSIGKILL=no，主管负责等待原生退出。此选择有意保留 Agent 并允许再次启动；若主管异常退出后原生进程仍在，进程扫描拒绝并起另一个控制器。这是本次关闭动作门范围内的取舍。
- 状态区分 WAITING、CONNECTED（收到新鲜原生状态）、STOPPING、STOPPED；CONNECTED 不代表获得起飞许可，现有 readiness_issues 单独展示。
- 安装器检查运行环境，拒绝覆盖非本包的同名服务；重复安装不重启现有活动服务。提供 status、logs、report、stop、disable；默认一条命令安装并显示结果。report 打包本服务限定的有限日志和配置。

## Boundaries

本包不接入视觉、不启用 RC 拨杆任务、不改原生状态机或保护阈值。后续 CH5 手动解锁需要原生状态机适配；现有 takeoff 仍是程序解锁，不能当成 A 方案已经实现。此前 VERTICAL_VELOCITY_INCONSISTENT 未解决。

验收：离线解包、服务单位语法、无终端持续待命、异常/退出/重复启动、关闭动作门及日志限额都有主机验证。没有设备远程连接，不能把主机验证表述成香橙派开机实测。
