# Stage 7：实际 ROS 2 Humble 构建与接口验收

## 结论与边界

在最终源码 `4c3c360941575049ebac78dbbf4ce58f098d93b6` 上重新创建了独立 build/install/log 目录，真实构建并验证了安装结果。以下所有步骤退出码均为 0。本子任务没有修改生产源码、没有执行生产控制器、没有连接 PX4、没有启动 Agent / systemd 服务、没有部署。

环境为本机 x86_64、Ubuntu 22.04 系列用户态、ROS 2 Humble，RMW 为 `rmw_fastrtps_cpp`。这不是 ARM64 构建，也不能证明相机性能、实机稳定性或飞行延迟。

## 实际执行结果

| 步骤 | 实际结果 | 秒 |
| --- | --- | --- |
| 全新 colcon 构建 `jikl_fc01_system` | 1 个包构建成功 | 45.690072 |
| 当前实际生成消息身份与本地 CDR | 5 / 5 类通过 | 0.504638 |
| 视觉状态适配器真实 ROS 通信 | 7 / 7 检查通过 | 4.581695 |
| 目标速度只读诊断真实 ROS 通信 | 11 / 11 检查通过 | 9.566871 |
| 被动 ROS 证据采集器生命周期 | 3 / 3 正常创建、采集、清理通过 | 4.653783 |
| 图隔离前后检查 | 6 / 6 通过 | 已计入通信步骤 |

以上是不同计数单位，不能简单相加为 Python unittest 数。

## 复核方法

- `build.command.sh` 从 `/opt/ros/humble` 与既有 px4_msgs underlay 开始，清空继承的 overlay 路径，新构建目录不存在旧产物。px4_msgs underlay 来自 `/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs`；本次未重建外部 PX4 消息包。
- 生产节点与配置检查器确实编译成 ELF，使用 readelf 静态读取；两者均未执行。构建启用 Release、并发 2、BUILD_TESTING=OFF；本步骤不是 C++ 单元测试，单元测试由主验收单独运行。
- `tests/ros_generated_interface_check.py` 来自当前源码，检查实际 ament prefix、Python 包、typesupport 路径、五个消息的字段/常量/默认值、非默认值 CDR 解码。没有调用 rclpy.init，也没有创建 ROS 节点。
- `VelocityCommand` CDR 重编码存在 4 字节已验证的对齐填充差异；两次解码后的字段严格一致。检查器只允许已按 CDR1 对齐规则定位的 padding 差异，不放宽消息字段相等条件。
- 真实 ROS 测试分别使用仅本机的隔离域 223 / 224 / 225，清除继承的 discovery server 与 DDS 配置。每组测试前后检查图，仅允许检查程序自己的临时节点及 ROS 元数据 topic。
- 视觉 smoke 启动本轮新安装的只读适配器，使用合成 Odometry / VisionHealth，检查转换、来源冲突撤销、UTF-8 字符串预算拒绝、新鲜配对恢复、只读参数、无 /fmu 发布者及 SIGINT 干净退出。
- 速度 smoke 启动本轮新安装的只读诊断器，检查真实 payload、QoS 兼容、重复/乱序、失效、重置/来源改变、多发布者、超时与退出快照，不产生控制权限。
- 采集器使用最终 `navigation_collect.py`，连续 3 轮仅订阅 9 个观察 topic，每轮均没有消息，退出时 Context 均已关闭；没有创建模拟 /fmu 发布者。这里只验证正常生命周期，异常路径的确定性替身测试由主验收单独统计。

## 源码与安装身份

- 构建前、构建后、验收结束的 240 个 tracked native 输入文件 SHA-256 一致：`build.source_before.json`、`build.source_after.json`、`build.source_final.json`。
- 8 个验收/被测脚本的前后 SHA-256 一致：`acceptance.source_before.json`、`acceptance.source_after.json`。
- 4 个安装 Python 工具和 5 个安装 MSG 文件均与当前源码逐字节一致：`installed_identity.json`。
- 结束复核时生产工作区干净，HEAD 仍是上述提交。本子任务没有本地提交。

## 可复核证据

本目录保留每步 `*.command.sh`、`*.result.json`、`*.log`、详细 `*.report.json`；汇总入口为 `verification.json`。图检查为 `*.domain_preflight.json` 与 `*.domain_postflight.json`。原构建及安装目录保留在缓存，不纳入仓库，也不部署。

不能由本次测试推导：ARM64 可用性或性能、真实视觉时间同步质量、外参/尺度正确性、PX4 实机闭环稳定性、负载突发时的最坏延迟、数学上的全状态安全证明。视觉控制是否启用应由配置/发布门的独立检查确认，不能仅依据接口通信成功。
