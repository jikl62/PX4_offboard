# 视觉接口补验与文件完整性核对（2026-11-11）

本轮对 `e825481` 继续审查和验证。没有发现源码或原飞行配置丢失，发现一处 ROS 测试首帧竞态并修复；另补齐了上一轮临时日志被清理后缺少的可复核记录。机器可读结果和关键原始输出保存在 [补验证据](evidence/20261111_vision_followup_verification.json)。

## 文件与功能完整性

对照视觉开发前 `950ac04` 的全部 379 个已跟踪文件，缺失为 0。374 个文件逐字节一致；其余 5 个是已说明的接口改动：`FC01_RELEASE.json`、消息生成/安装的 `CMakeLists.txt`、`package.xml`、`navigation_report.py` 和报告测试。`8b57077` 的 391 个已跟踪文件也全部存在。

原飞行保护清单的 65 个文件继续逐字节一致，包括 Runtime、节点、RC/权限、垂直反馈保护、正式配置和服务入口。新离线预览头文件只被测试引用，没有进入生产控制链。目标高度 `1.5 m`、hover `0.50`、reserve `0.04`、thrust max `0.80` 和 `visual_navigation_enabled=false` 保留。

当前打开的 `native/config/ground.yaml` 是禁止飞行的地面配置，没有被本轮视觉开发改动。正式导航服务在 `navigation_service.candidate_config_path()` 选择 `flight_test.yaml`，再由 `load_profile()` 读取并组装运行参数；它不从 `ground.yaml` 加载自动起飞配置。

核对了原始 `VelocityCommand.msg` 十个字段、默认值和顺序，消息生成以及 adapter/诊断/helper 的安装项均齐全。报告收集默认 JSON、日志和 `.log.1` 三个文件；自定义输出目录不被默认报告扫描，这个限制已在接口文档中明确。

## 本轮发现与修改

1. `ros_velocity_command_smoke.py` 的两个首帧等待条件，对合法初始快照 `last_message=null` 执行 `.get()` 会抛 `AttributeError`。首次数据处理慢于快照轮询时，测试可能假失败。终态检查也有同类空值问题。仅修复这三处测试判定，让空消息返回 false 并继续等待，终态缺消息时给出原有明确断言。
2. 接口参数表重复列出 `expected_source_id`，删除重复行。
3. 上一轮 `/tmp` 构建、安装和运行日志已不存在。此次在 `/home/jikl62/.cache/fc01-followup-20261111/` 重新做干净构建、当前测试和安装后通信验证，并归档日志与独立复现源码；关键原始输出同时写入版本控制中的补验证据。未将历史日志摘要当作本轮执行结果。

本轮没有修改生产控制、诊断接收器或状态校验源码，没有重新生成 `dist/` 安装包。

## 实际验证结果

| 验证集合 | 结果 |
| --- | --- |
| VelocityCommand 接收器 Python | 41 项通过 |
| 视觉状态校验 Python | 41 项通过 |
| adapter、设置、报告、起飞配置、服务和反馈/诊断 Python | 111 项通过 |
| 载荷及构建证据 Python | 22 项通过，ARM64 部分为无害 fixture |
| Python 合计 | 215 项通过，0 失败 |
| C++ 安全及视觉回归 | 11 个程序通过，其中预览 458 个检查 |
| 新预览 UBSan | 458 个检查通过 |
| 干净 ROS 2 Humble 构建与安装 | 通过，编译 stderr 为空 |
| 安装后三个 Python 工具 | 与源码 SHA 一致，helper 可从安装目录导入 |
| 真实 ROS 速度指令通信 | 11 个检查通过，使用安装后的诊断程序；修复测试后在 Domain 203 复核 |
| 真实 ROS Odometry 适配 | Domain 202 通过，包含坐标转换与多来源撤销 |

215 是 11 个 Python 模块的当前去重总数。前次 203 和上轮 149 分别代表不同版本、不同覆盖集合，不能把 149 与 203 的差额解释成删除测试。其他审查者重复执行的 66 项，以及修复前后重跑的 ROS 检查，均不重复累计。

另有独立边界验证：7 组接收器撤销/恢复场景、实际烟测条件对空/正确/错误消息的 9 组回放、2,000 个随机半正定协方差、2,000 个明确不定矩阵，以及使用独立四元数公式核对的 5,000 组旋转与限幅。它们与单元测试分开计数，未发现新增实质问题。

## 复核命令与记录

完整 Python 组合：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_velocity_command_diagnostic test_vision_state_validation \
  test_vision_adapter test_vision_settings test_navigation_report \
  test_takeoff_configuration test_navigation_service \
  test_navigation_feedback_freshness test_navigation_diagnostics \
  test_vertical_diagnostic_bundle test_navigation_build_evidence
```

实际执行按 41、41、111、22 四组保存日志，避免并行审查重复计数。C++ 按原开发记录的 11 个程序严格编译；命令参数、逐程序输出、源码 SHA 与两组 ROS 结果见补验证据。

干净构建目录为 `/home/jikl62/.cache/fc01-followup-20261111/root_e825481/`。使用 `/opt/ros/humble/setup.bash` 和原有 px4_msgs SDK underlay，以 `colcon build --base-paths native/src/jikl_fc01_system` 构建；build/install/log 均指向该外部目录，未复用上一轮 `/tmp` 产物。构建的两份控制器可执行程序仅编译安装，没有执行。

归档 `FC01_vision_followup_20261111_logs.zip` 包含本轮日志、JSON 记录与独立复现源码，具体位置和 SHA 见补验证据。仓库内证据还包含关键原始输出，即使外部缓存目录以后清理，仍可复核本轮结论。历史验证文件保持历史含义，当前源码摘要由 `FC01_RELEASE.json` 另行更新。

本轮是 x86_64 主机软件验证；没有 ARM64/Orange Pi 验收、相机真实输入、PX4 EKF 注入或实际起飞测试。只启动了 localhost 隔离的合成视觉发布者、独立 adapter 和诊断节点，没有启动生产控制器、Agent、串口或 systemd 服务。
