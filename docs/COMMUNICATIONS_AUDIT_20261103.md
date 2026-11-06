# 2026-11-03 通信证据审计与只读采集

**2026-11-06 映射审计更正：** 用户最新物理记录仍是现用 **香橙派 UART2 `/dev/ttyS2` ↔ FC01 USART6/RX6-TX6**、候选 **香橙派 UART6 `/dev/ttyS6` ↔ FC01 UART4/TELEM3**；但 FC01 NSH 明确显示 uXRCE 启动在 `/dev/ttyS3`，本地板级源码将其映射为 UART4。物理线束与运行/源码证据冲突，不能把 UART4 标为闲置，也不能修改现用参数。详细证据、TELEM4 命名区别与只读步骤以 [双串口审计](FC01_DUAL_UART_20261105.md) 为准。

结论：历史完整话题负载确实超过115200容量；现有资料还不能证明当前故障是串口饱和，也不能把UART6加第二个Agent当成已经支持的DDS分流方案。先绑定实际部署、采集同步窗口，再决定是否需要第二链路。本轮没有SSH、改服务、改飞控参数、打开串口或改接线。

## 证据时间与固件身份

| 来源 | 可证明的内容 | 不能据此推出的结论 |
|---|---|---|
| [10/26实测频率负载报告](/home/jikl62/FC01_DDS_V3_DEV/V3_HOST_TEST/reports/FC01_DDS_Measured_Load_Report_20261026.md:34) | 当时25个输出定义在原10ms转发上限下，未转义需求65,989 B/s；115200/8N1单方向容量11,520 B/s，需求为5.73倍。仅电池100Hz或本地位置84Hz就分别超过整条TX容量。 | 这是历史uORB快照、锁定CDR尺寸及模型预算，不是当前实机线上吞吐。uORB频率、DDS提交量、UART交付量不同。 |
| [10/26串口机制报告](/home/jikl62/FC01_DDS_V3_DEV/V3_HOST_TEST/reports/FC01_DDS_115200_Investigation_Report.md:25) | 旧版O_NONBLOCK传输遇到短写/EAGAIN可能丢弃帧后缀；真实Agent2.4.2分帧器的受控复现中，旧帧某些截断位置能使后一条完整Ping未被识别。 | 模拟数不是现场丢帧数；`expired`不等于Agent漏答数，也不能定位故障层。未复刻现场11 attempts/5 replies/6 expired/1 send_error完整因果链。 |
| [V3主机验收报告](/home/jikl62/下载/V3_TEST_REPORT.md:50) | 相同历史源频率下，单独整帧保留仍超载；显式限流后模型约7,998 B/s、179/179 Ping。小队列、事件风暴、极端转义与高可靠流压力仍有失败。 | 180秒是虚拟时间；7,998 B/s不是当前实机负载、最坏上界或飞行验收。 |
| [10/27原生构建日志](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD.log)及[生成桥接头](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/build/damiao_dm-fc01_default/src/modules/uxrce_dds_client/dds_topics.h:191) | 10/27 00:49原生1216/1216成功，FLASH1,786,940 B/1792 KiB=97.38%；[CMakeCache](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/build/damiao_dm-fc01_default/CMakeCache.txt:423)为`diagnostic`，电池500ms，姿态/位置50ms，IMU/里程计1000ms，[实际init](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/build/damiao_dm-fc01_default/src/modules/uxrce_dds_client/dds_topics.h:406)使用逐话题间隔。 | 更早V3报告“尚未原生编译”属于前一阶段；不能继续当最终状态。生成的`.px4`与当前实机镜像逐字节身份仍未确认，不能只凭PX4 git hash认定补丁和编译profile一致。 |
| [现仓历史用户观测](../README_zh.md:75) | 曾测得姿态/位置20Hz、状态2Hz、落地1Hz，30秒源样本推进，480/480 Ping且没有新增重连。 | 不是11/3实时状态，也不能替代应用闭环、视觉接入或实际离地证据。 |
| 10/31 ULog：`/home/jikl62/桌面/log_96_2026-10-31-17-52-00.ulg` | 同轮主审计读取到约17.18秒、PX4 git hash前缀07983bf、UXRCE配置103/Domain0/TELEM3 115200，以及takeoff detected后failsafe。详细因果由主审计分析。 | ULog不能单独证明当前OrangePi源码/二进制、视觉运行、实际离地或UART饱和。本文未把takeoff detected解释成独立离地测量。 |

115200 UART为全双工时，两方向各有11,520 B/s线速预算，不能把双向流量相加后与一条单方向容量比较。XRCE分帧、转义、Ping、时间同步、实体创建与可靠控制还有开销；ROS序列化尺寸也不等于UART线上字节。

## 部署、SDK、QoS与应用等待

当前入口约定仍是香橙派`/dev/ttyS2 @115200`、ROS Domain0；[navigation_boot.sh](../navigation_boot.sh:4)加载Humble和原工程`install_r632`。FC01端不能由香橙派入口推断。用户物理记录指向 USART6，但现场 DDS 启动端点为`/dev/ttyS3`，本地源码映射为 UART4；两类证据冲突，不能据此改参数或声明候选空闲。这些信息仍不能证明实际systemd在该目录、使用该二进制或加载了该SDK。

[节点](../native/src/jikl_fc01_system/src/fc01_system_node.cpp:54)固定`VehicleStatus`版本1等消息，状态订阅名是`/fmu/out/vehicle_status_v1`。已有[SDK构建记录](navigation_repair_20261031.md:35)显示通用`px4_msgs`曾缺少实际所需字段，必须绑定实际部署的SDK，不能任意换成最新消息包。

[飞控输出QoS源码](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/src/modules/uxrce_dds_client/utilities.hpp:79)为TRANSIENT_LOCAL/BEST_EFFORT；输入为VOLATILE/BEST_EFFORT。应用反馈使用[SensorDataQoS、keep_last10](../native/src/jikl_fc01_system/src/fc01_system_node.cpp:75)。话题存在但无匹配数据时，类型、版本、Domain与端点QoS都应单独检查。没有当前实机graph，实际节点和端点数未知。

[launcher](../launcher.py:198)仅复用同用户、同串口、115200的单Agent，遇到多个Agent或未知占用会拒绝启动。进程冲突属于部署问题，不能归为线速不足。飞控模板按uORB更新主动转发，没有按ROS订阅者数乘发的逻辑；“节点多”本身不能证明FC01到Agent的UART字节倍增。

原生[主循环](../native/src/jikl_fc01_system/src/fc01_system_node.cpp:720)按`spin_some → tick → sleep_until`运行，尚无当前循环耗时证据。[日志](../native/src/jikl_fc01_system/include/jikl_fc01_system/async_log.hpp:19)已使用有界后台队列及非阻塞输出。终端停更、条件拒绝、反馈样本超龄与真实串口停顿需要分开判断；Ping持续成功也不保证`timestamp_sample`推进或控制循环及时执行。

## UART6可用性结论

| 已核实源码事实 | 限制 |
|---|---|
| [板级配置](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/boards/damiao/dm-fc01/default.px4board:3)的`CONFIG_BOARD_SERIAL_URT6`是FC01侧`/dev/ttyS5`；TELEM3是`/dev/ttyS3`。 | 不能把FC01设备名当成香橙派设备名。连接器针序、电平、实际板版本及空闲情况未知，不从MCU GPIO定义推断接插件针序。 |
| [生成启动映射](/home/jikl62/FC01_DDS_STARTUP_BUILD_20261026/build/damiao_dm-fc01_default/etc/init.d/rc.serial_port:2)把串口选择值6映射`/dev/ttyS5`及`SER_URT6_BAUD`。 | 这是选择现有单条链路的映射证据，本文没有设置该参数。 |
| [V3最终NuttX配置](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/build/damiao_dm-fc01_default/NuttX/nuttx/.config:1008)的USART6为RX180B、TX256B、默认57600/8N1、无流控；板级未启用USART6 DMA。现有UART4/TELEM3为3000B缓冲及RXDMA。 | UART6不是等价缓冲与DMA条件的替换接口；实际运行参数仍未知。 |
| [uXRCE类](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/src/modules/uxrce_dds_client/uxrce_dds_client.h:59)继承ModuleBase；[基类start](/home/jikl62/FC01_DDS_V3_NATIVE_BUILD/platforms/common/include/px4_platform_common/module.h:199)在运行时返回`Task already running`。 | 读取的定制固件没有双串口uXRCE实例或逐话题跨串口分片能力。只在Linux多开Agent不能增加飞控端的第二个DDS发送源。当前板上镜像能力仍需身份绑定。 |

本次审计不改用户报告的现用链路，也不启用候选 UART4。先解决物理线束与`/dev/ttyS3`运行端点的冲突，再排除 SDK/Domain/QoS/进程冲突与应用等待，最后测当前双向字节和样本年龄。只有证据显示容量不足时，才评估扩容或明确的双链路桥接设计；第二链路还需核实硬件、端点身份、话题归属与固件资源。图像/视觉内部数据也不应未经预算整体送入飞控串口。

## 只读采集工具

新增[采集器](../navigation_collect.py)的默认路径只需Python标准库，不导入ROS，不执行现有启动入口，也不连接控制器IPC：

```bash
python3 navigation_collect.py --output ~/FC01_communications_20261103
```

输出目录必须尚不存在。默认观察`fc01-navigation.service`和`/dev/ttyS2`；可用`--service`、`--device`或`--project-root`指定已有部署。它只运行有超时的`systemctl show`和`fuser -v`，读取`/proc`及常规文件，写入报告目录的`report.json`。不会启动/停止Agent、控制器或服务，不打开串口、不设置波特率、不修改飞控参数，不创建任何`/fmu/in`发布者。

报告保存实际service的WorkingDirectory、ExecStart、相关Environment、EnvironmentFiles路径、active/substate/PID、unit及drop-in路径；环境只保留ROS/SDK/导航相关键。按WorkingDirectory采集源码/入口设置哈希，记录实际进程argv中的`--params-file`、当前`/proc/PID/exe`哈希、可见UART FD、Domain/SDK环境及SDK package.xml版本。常用runtime/base配置与候选安装二进制也分别标记为磁盘文件；磁盘文件存在不证明当前进程加载了它。无权限、缺工具、文件变化或缺文件均明确记录unknown，不伪造“通过”。不采集完整进程环境或无关凭据。

需要被动ROS窗口时，在现有SDK可用的普通用户终端运行：

```bash
source /opt/ros/humble/setup.bash
source /实际工程路径/install_r632/local_setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 python3 navigation_collect.py \
  --output ~/FC01_communications_ros_20261103 --ros-seconds 10
```

Domain应与报告中实际进程一致；示例0仅对应当前工程约定。工具继承调用终端环境，**不会自动执行service中的脚本、切换SDK或假装使用服务环境**。两套环境都会记入报告供比较。ROS导入/运行出错或超时也写unknown，不启动Agent补救。

ROS采样在独立子进程内，窗口可选0..30秒（默认0），总等待上限为窗口加12秒。仅订阅9个关键`/fmu/out`话题，保存`ros_topics.json`：接收Hz、接收间隔、消息与原始样本时间年龄、重复/回退计数、CDR序列化尺寸、节点清单、发布/订阅端点数及QoS。观察者会增加每话题一个订阅者；发现阶段包含在窗口内，短窗口可能低估稳态Hz。消息年龄采用主机墙钟减来源时间戳，未证实时钟一致时不能解释成真实传输时延；负值和缺失时间戳保留，不当成0延迟。CDR B/s不是UART吞吐，短期有消息不等于长期稳定。

这份报告不能读取FC01 NSH状态或物理UART字节。下一次同窗取证仍需FC01开始/结束各一次`uxrce_dds_client status`、只读镜像/参数核对及uORB频率。普通状态仍不足以定位时，再对既有Agent限时附加串口read/write跟踪并记录扰动；Agent write成功只说明本地接受。已有逻辑分析仪可被动同窗记录，缺少FC01 RX证据时不强行区分返程丢失与解析/调度失败。

采集器9项局部回归先失败后通过，覆盖无ROS帮助、缺工具/限时退出、真实文件/运行镜像指纹、service目录与调用目录区分、敏感环境排除、缺失unit、Domain不匹配/部署未知、样本年龄/重复及CDR预算口径。真实Humble/原SDK的隔离Domain214合成烟测仅在`/fmu/out/vehicle_attitude`建立测试发布源，采集29条、样本年龄50.42–50.71ms、ROS序列化56B/条、发布端点数1；与锁定飞控ucdr内容49B口径不同。该测试没有连接Agent或飞控，不代表香橙派或FC01实机通信验收。开发机报告位于`/tmp/fc01_communications_collect_ros_messages_20261103/report.json`。
