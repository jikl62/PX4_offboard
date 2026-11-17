# 阶段 3：构建来源与参数合同独立复核

复核时间：2026-11-16。开始 HEAD `8767608f79a56e346d5654228770f948841fe0e3`。只读取文件、Git 状态/差异与 ELF 信息；未编译、启动 ROS/控制器、连接硬件或改动 Git/生产文件。详细逐文件结果：[BUILD_AUDIT.json](BUILD_AUDIT.json)。

## 结论

**本次检查未发现构建来源或参数夹具阻断项。** 当前开发机安装 ELF 与阶段 2 归档哈希一致，生产输入与已验证字节一致，安装头文件/消息定义与当前源码匹配；Stage 3 YAML 是已采集板端 runtime 的逐字节副本，五个推力参数、1.5m 目标和视觉关闭设置均一致。可在明确的本地合成 ROS 测试范围内复用该安装 ELF，并记录其真实哈希/架构；这不生成新发行身份，也不代表已部署新版 ARM64 程序。

## ELF、源码与安装产物

`docs/investigations/20261116_stage2/evidence/ROS_BUILD.json` 指向：

```text
/home/jikl62/.cache/fc01-stage2-20261116/ros/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node
SHA-256 c14dbd9548217df28ceaf46ddc5de57b94a30e8b49dd9af9582668353defc0c0
ELF 64-bit x86-64 PIE, BuildID 72574494d98ff42b3c179499b7e8afe55343af99
```

本次从实际 ELF 重新计算的 SHA 与记录相同。该程序是 **x86-64**，不能作为 Orange Pi AArch64 安装包或板端运行验证。

- `ROS_BUILD.json` 中全部 **47 项 production_sha256** 与当前文件相符；清单中也包含 `.bak`/历史副本，不把它们全称为实际编译单元。
- 当前 `native/src/jikl_fc01_system` 与最终生产提交 `e8652ad686c52502ea877fd23b5805d6b64e0da6` 的 Git 差异为空。HEAD 后续文档提交没有改变该目录生产内容。
- 从实际编译依赖文件 `.o.d` 检查到的 **36 项工作区依赖**全部被输入摘要覆盖，且摘要相符。安装目录中的 **37 个源码头文件**与当前 include 树逐文件一致；该数量包括已有备份头，不暗示全部被编译使用。
- 安装 share 目录中的 **5 个 .msg 定义**与当前源码一致。本次不重新加载 type support；历史加载成功仅按阶段 2 记录陈述。
- `PROTECTED_FILES.json` 中 **20 项**配置/关键模块与当前相符；`regression/results.json` 的 after.files **200 项**当前摘要均一致。没有将历史测试通过当作本次重跑。

构建目录 CMakeCache 真实 home directory 指向本工作区 `native/src/jikl_fc01_system`，Release，`/usr/bin/c++`；rclcpp 来自 `/opt/ros/humble`，px4_msgs 来自 `/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs`。flags.make 指向当前源码 include 和该 PX4 SDK；command.log 记录 CMake build/install 均退出 0。摘要、依赖与既有构建证据支持输入/产物绑定；本次没有做可复现构建，也未认证系统工具链及依赖库的完整供应链哈希。

## 板端 runtime 与两份测试夹具

实际读取报告 ZIP `/home/jikl62/桌面/FC01_navigation_report_20261114_093631_680121.zip`，SHA-256 为 `ca2a2cc685995169526e08658d552103c4419bccb08c8cb2e7f1a4cf4a7eac4f`，与阶段 1 归档一致。ZIP 内 `runtime.yaml`、阶段 1 `evidence/runtime_20261114.yaml`、Stage 3 `tests/fixtures/stage3_runtime_20261114.yaml` 三者内容相同，SHA-256 均：

```text
800202615c8d45e4627c79d308ea4097f15c545ebb45181eaf8451419818f089
```

该 YAML 含 **72 个参数**。`native/tests/stage2_20261116_captured_config.hpp` 中带参数注释的 **64 个 attitude 配置赋值**逐项与此 YAML 一致；另外的通用运行项并未被假称为同样的 64 项。关键参数同时与当前 `attitude_ros_parameters.hpp` 的实际映射核对：

| runtime 参数 | 当前生产成员 | 板端/夹具值 |
| --- | --- | --- |
| aa_hover_thrust | c.vertical.hover | 0.50 |
| aa_takeoff_reserve_thrust | c.takeoff_reserve_thrust | 0.04 |
| aa_thrust_min | c.vertical.minimum | 0.12 |
| aa_thrust_max | c.vertical.maximum | 0.80 |
| aa_thrust_slew | c.vertical.slew | 0.40 |
| rc_target_height | c.rc_target_height | 1.5 m |
| visual_navigation_enabled | c.visual.enabled | false |

板端 YAML 还保留 `flight_actions_enabled=true`、`arm_enabled=false`、`takeoff_enabled=true`、`rc_navigation_enabled=true`、`bench_arm_only_enabled=false`、`flight_enabled=false`、`autonomous_failsafe_confirmed=false`、`px4_land_fallback_confirmed=false`。`flight_enabled=false` 不能单独解释为该人工 RC 路线绝不会产生非零动力；应以实际权限合同和隔离运行环境判定测试范围。

本次读取的 Stage 3 harness 在运行时只覆写 socket_path、fc01_config_path、fc01_config_source_path 与 fc01_entry_version 四项本地路径/测试身份，并将实际 YAML 写入输出目录；未见它覆写上述推力、目标、视觉或权限参数。这里是源码静态检查，不宣称已独立执行 harness。板端旧 `20261112.1` 身份仍只属于历史采集版本，不能因为复制其参数而称新代码已经板端发行。

## 行为等价与报告声明边界

阅读了 `STAGE2_REPORT_zh.md`、`COMMITTED_SOURCE_CHECK.json`、差分元数据和最终独立复审。报告明确分开：旧行为严格差分、18 个新增只读诊断键、已确认离地期限修复的有意控制阶段变化。过期运动证据用例旧版进入 Ascend、修复版留在 Ramp，不能说所有输入下全行为等价。

报告中的 Core/Runtime/发布字节一致和合成节点生命周期属于已执行测试集的结果。当前源码字节与历史测试输入一致，支持继续引用这些**限定样本范围**的历史结果；并不构成所有执行路径安全性、动力学稳定或最新硬件异常根因的证明。报告已有明确限制：阶段 2 未实测新版真实 DDS、ARM64、传感器质量、飞行收敛、失联退出延迟及 TSan，且本地 publish 返回不等于 PX4 实收；本次未发现将这些项目冒充已通过的表述。Stage 3 若新增真实本地 DDS 验证，应标明“x86 ROS 本地合成 peer”，不能扩写成真实 PX4/Agent/UART 或实飞验证。

姿态 reset、垂向异常、心跳最终上下文和失联保护继续保留；未发现需要放开保护才能做本地验证的构建或参数依据。range 的只读观测不认证原始 MTF 设备、不保证与 EKF z 独立。Position 本地档案缺对照片段也不否定用户现有 Position 已稳定的现场基线。

本复核只确认当前可见的文件与记录一致。没有更新发行、覆盖板端目录、启动任何飞行程序，也没有重跑阶段 2 全回归。
