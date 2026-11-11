# FC01 视觉接口开发与验证记录（2026-11-11）

本轮完成可独立验证的消息、接收诊断、状态校验和控制预览。生产导航服务继续关闭视觉导航；新接口没有进入飞行发布链。接口合同与启动方法见 [VELOCITY_COMMAND_INTERFACE.md](VELOCITY_COMMAND_INTERFACE.md)，实际结果见 [机器可读证据](evidence/20261111_velocity_interface_verification.json)。

## 源码与兼容边界

开发基于 `release/fc01-recovery-v5-20261110` 的 `950ac04`。工作开始时 Git 工作区为空；交接材料的 `b388f26` 是它的祖先，没有回退或覆盖后续修复。

| 文件 | 交付内容 |
| --- | --- |
| `native/src/jikl_fc01_system/msg/VelocityCommand.msg` | 保留交接文件十个字段的名称、类型、顺序和默认值；说明正常导航、FRD 方向和无效帧撤销语义 |
| `native/src/jikl_fc01_system/CMakeLists.txt` | 生成新消息；安装独立诊断脚本及状态校验 helper，复用现有依赖 |
| `native/tools/velocity_command_diagnostic.py` | 可配置话题/QoS、完整接收检查、可失效缓存、来源绑定、事件与有界日志；不创建飞控 publisher |
| `native/tools/vision_state_validation.py` | 复用原 Odometry 转换；增加显式时间映射、质量/连续性/epoch、原始协方差校验 |
| `native/src/jikl_fc01_system/include/jikl_fc01_system/velocity_command_input.hpp` | 独立状态/目标结构、FRD→NED、速度误差、复用外环限幅及姿态预览；未被 Runtime 包含 |
| `navigation_report.py` | 原有只读报告附加收集诊断 JSON、日志与默认轮转日志 |
| `tests/test_velocity_command_diagnostic.py`、`tests/test_vision_state_validation.py`、`native/tests/test_velocity_command_input.cpp` | 纯计算和异常边界回归 |
| `tests/ros_velocity_command_smoke.py` | 隔离本地真实 ROS 类型、序列化、收发、QoS、来源/断流和无控制端点验证 |
| `tests/benchmark_vision_interfaces.py` | 真实生成类型的 CDR 大小与主机处理耗时测量，不建立 ROS 图 |
| `tests/test_navigation_report.py` | 新诊断文件收录、截断及符号链接边界 |
| 本文、接口文档和证据 JSON | 使用合同、真实结果、限制与对接事项 |
| `FC01_RELEASE.json` | 更新源码/载荷哈希并附本轮开发验证引用；不把历史验证记录冒充本轮结果 |

逐字节对照基线确认：`runtime.hpp`、`fc01_system_node.cpp`、`receive_events.hpp`、`vertical_feedback_guard.hpp`、原 Odometry adapter、正式高度控制 YAML、导航服务与设置均未变化。正式服务仍采用 RC 人工解锁与现有 Kill/Failsafe/权限检查；目标高度 `1.5 m`、hover `0.50`、reserve `0.04`、thrust max `0.80` 保留。

现有 `VisionState` / `VisionHealth` / `NavigationCommand` 的格式不变。新速度消息采用 **`jikl_fc01_system/msg/VelocityCommand`**，默认可配置话题 **`/fc01/vision/velocity_command`**，方向为视觉制导→独立诊断接收器。`schema_version` 的 `1` 是默认字段值；接收器拒绝其他版本。双方须使用同一完整 ROS 类型并重新构建消息包。

`Odometry` 提供实际状态，目标速度始终单独保存。新接收器没有转发到 NavigationCommand；新状态 helper 也没有自动替换现有 ROS adapter。C++ Preview 使用已有候选外环数学工具，输出只供离线核对，没有创建 `attitude::Output`、PX4 控制消息或权限租约。垂向目标展示值未接入原高度控制器。

## 验证方法

构建在当前 **x86_64 / ROS 2 Humble / Python 3.10** 主机完成。px4_msgs 使用工作区已有 SDK 安装前缀；没有下载或修改飞控固件。构建产物、详细日志放在临时目录，未混入源码或发布安装包。

本轮实际构建命令（安装路径按本机记录）：

```bash
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
FC01_VISION_BUILD=/tmp/fc01_velocity_development_kdwvCj
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m colcon \
  --log-base "$FC01_VISION_BUILD/log" build \
  --base-paths native/src/jikl_fc01_system \
  --build-base "$FC01_VISION_BUILD/build" \
  --install-base "$FC01_VISION_BUILD/install" \
  --packages-select jikl_fc01_system --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source "$FC01_VISION_BUILD/install/jikl_fc01_system/share/jikl_fc01_system/local_setup.bash"
```

Python 相关回归：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_velocity_command_diagnostic test_vision_state_validation \
  test_vision_adapter test_vision_settings test_navigation_report \
  test_takeoff_configuration test_navigation_service \
  test_navigation_feedback_freshness test_navigation_diagnostics
```

C++ 以 `-std=c++17 -O1 -Wall -Wextra -Werror -pedantic -pthread` 编译和运行以下 11 个程序：`test_velocity_command_input`、`test_vision_navigation`、`test_visual_runtime`、`test_attitude_runtime`、`test_authority_runtime`、`test_rc_navigation`、`test_rc_before_ready_recovery`、`test_publication_runtime`、`test_receive_schedule_safety`、`test_m1_safety`、`test_authority_boundary`。逐程序输出记录在证据 JSON。新增程序也会被原有 `native/tests/CMakeLists.txt` 的 `test_*.cpp` 自动发现。

新增预览的 UBSan 命令：

```bash
g++ -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Werror \
  -fsanitize=undefined -fno-sanitize-recover=undefined \
  -I native/src/jikl_fc01_system/include \
  native/tests/test_velocity_command_input.cpp \
  -o /tmp/fc01_test_velocity_command_input_ubsan
/tmp/fc01_test_velocity_command_input_ubsan
```

真实 ROS 检查分别在 localhost 的隔离 Domain 188 和 189 运行；速度检查使用**实际安装后的脚本**，验证构建及安装路径：

```bash
ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 tests/ros_velocity_command_smoke.py \
  --diagnostic "$FC01_VISION_BUILD/install/jikl_fc01_system/lib/jikl_fc01_system/velocity_command_diagnostic.py" \
  --report "$FC01_VISION_BUILD/ros_velocity_smoke_final.json"
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 tests/ros_vision_smoke.py
```

这是合成发布者与独立 adapter/诊断之间的真实 DDS 通信。没有启动 FC01 控制器、MicroXRCEAgent、串口、systemd 服务或真实飞行。速度节点仅有 ROS 自带 `/rosout` 和 `/parameter_events` publisher；Odometry 检查确认转换、来源冲突撤销以及无 `/fmu` 控制发布端点。

最终结果如下。相关 Python 组合先执行 170 项，修复边界后仅重跑受影响的接收器与状态模块；以下数量按最终版本去重，未把重复执行累计为新增测试。

| 验证 | 通过 | 失败 |
| --- | ---: | ---: |
| VelocityCommand Python | 33 | 0 |
| 新状态校验 Python | 37 | 0 |
| 现有 adapter、设置、报告、起飞/服务/反馈安全 Python | 111 | 0 |
| 载荷完整性及构建证据 Python | 22 | 0 |
| Python 合计 | 203 | 0 |
| C++ 程序 | 11 | 0 |
| 其中新增预览检查（已包含在上行） | 409 | 0 |
| 新预览 UBSan | 1 个程序 | 0 |
| 真实 ROS 通信 | 2 组 | 0 |

载荷及构建证据回归命令：

这一组使用临时目录和无害构建器桩检查复用/篡改检测，不代表执行了 ARM64 编译；输出中的 ARM64 标记属于合成 fixture。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_vertical_diagnostic_bundle test_navigation_build_evidence
```

Humble 完整构建和最终安装均通过，编译日志未见 warning/error；三个安装后工具逐字节匹配当前源码，状态 helper 可从安装目录正确导入。最后检查包括 `git diff --check` 和源码载荷哈希一致性。

## 审查中发现并已修复的问题

开发、主审与两个并行审查者交叉检查了源码及可复现边界，并补充相应测试：

1. 诊断节点的方法名与 `rclpy.Node.context` 属性冲突，真实 ROS 启动失败。已改为 `receive_context`，实际安装节点可启动和收发。
2. 被拒绝的未来 stamp / 非法大序号曾污染可信水位，阻止正常消息恢复。现在仅通过检查的消息或合法的显式撤销推进水位。
3. 健康新包曾覆盖旧接收时间，使定时器未先执行时漏记断流。现在接收回调也先记录旧缓存到期；正常尾包不能消除异常计数与最后异常。
4. 状态跳变被拒后若删除可信基线，重复异常位置可能被洗成正常状态。现在保留最后可信连续性基线，只有明确新 epoch 或连续轨迹可恢复。
5. 状态查询时钟回退曾可绕过接收时钟记录。现在接收/查询共享时钟水位，并在无缓存时继续记录。
6. 切换估计器 reset / 重定位后，旧 epoch 回放曾被当作新 epoch。现在保留有界可信历史，拒绝退役 epoch，历史不被非法样本污染。
7. 按整个协方差矩阵的最大元素统一容差会让极大方差掩盖其他轴的不定子块。现在按各轴标准差归一化，并检查对称性、Cauchy 必要条件和半正定分解。
8. 查询返回的 covariance 诊断曾共享内部缓存字典。现在深拷贝，调用方修改返回结果不会污染后续诊断。
9. C++ 预览补齐三维姿态连续性、q/-q 等价、采样偏差及交替回调掩盖过期空窗的检查。
10. 诊断来源历史达到容量上限时曾淘汰旧水位，可能让旧包重新通过。现在明确拒绝超额新会话，保留原有防重放证据及已有会话恢复能力。
11. reset / 重定位曾允许同一已确认时域的测量 stamp 倒退。现在仅允许位姿/序号重建基线，测量时间跨周期严格前进；被拒样本不占用新周期身份。

这些修复针对本轮新增模块；它们不宣称修复现有飞行核心的历史问题。

## 资源与时序

运行 `tests/benchmark_vision_interfaces.py --samples 5000 --output /tmp/performance.json` 会测量真实生成类型和纯计算代码，不创建 publisher/subscriber。原始数值、分位耗时、CPU 时间及限制列在证据 JSON；它们是本机负载下的样本统计，没有替代 Orange Pi 的调度或最坏执行时限测量。

| 本机操作（每项 5,000 次） | 平均墙钟耗时 | p99 |
| --- | ---: | ---: |
| Command 序列化 | 3.19 μs | 3.49 μs |
| Command 接收检查（显式合成策略） | 14.20 μs | 16.24 μs |
| 既有 Odometry 转换 | 50.87 μs | 56.48 μs |
| 状态校验，含转换与两个原始协方差检查 | 374.06 μs | 400.18 μs |

状态校验的平均 CPU 时间若按假设 50 Hz 估算，约占本机单核 1.87%；其中已包含 Odometry 转换，不能重复相加。进程峰值 RSS 59,204 KiB，包含 Python/ROS 导入，不能当作新增模块的净内存占用。分位数包含计时器开销，既不是设备端结果，也不是最坏时限。

消息 CDR 大小受本次字符串长度影响：VelocityCommand 108 B，Odometry 732 B，VisionHealth 184 B，VisionState 812 B。假设四条本地视觉 Topic 各 50 Hz，共 91,800 CDR B/s，新增 PX4 UART 数据为 0。这个频率是计算场景，不是已约定的传感器频率。

现有 20 Hz attitude+heartbeat 为 1,360 CDR B/s；若将来另行实现 50 Hz VehicleOdometry→PX4，示例数据为 6,000 CDR B/s。115200 baud、8N1 的原始单向线速容量为 11,520 B/s，但还要容纳 XRCE/RTPS/串口开销和原有遥测。因此这些数值不能证明当前串口余量，也不能推出本地视觉消息必然需要提高波特率。

诊断采用默认 depth=5、volatile、best effort（可显式改 reliable），单线程 ROS executor；接收与快照回调串行，无共享控制器锁。pending events 最多 128 条，来源历史和日志均有界，丢弃计数保留。默认每份日志 256 KiB、保留一份备份、普通日志最短间隔 0.2 s；异常在节流期间会合并保留。DDS 字符串字段遵守原协议而未改变 wire bound；本地接收后对身份文本做 256 字符限制，因此这不是针对任意恶意 DDS 流量的进程内存配额。

磁盘写入与 DDS 发布者计数查询发生在独立诊断进程，不运行在生产控制回调。处理耗时测试不含这两项或发现/排队时间。新状态校验是独立纯函数对象；C++ 预览只用于离线，不占生产控制周期。未来绑定仍须在目标设备上测量回调排队、磁盘延迟、循环 deadline 和高负载行为，不能将本次主机均值当作实时保证。

## 仍需外部条件的对接

- 视觉方同步完整消息包、实际 Topic / QoS / 发布频率和唯一会话标识。
- 提供真实 Odometry+Health 配对数据，确认测量时间、时钟映射、重置/重定位语义、质量与协方差声明；不能用零值补未知状态。
- 完成相机/机体外参、杆臂、尺度与世界航向对齐，并用真实数据确认方向和连续性阈值。
- 在 Orange Pi ARM64 上重新构建并测量 CPU、RSS、端到端延迟与队列；本轮未作 ARM64 验收，也未重新生成旧 `dist/` 安装包。
- 后续受控闭环联调需明确运动权限、目标有效期、退出策略及最终发布检查；本轮仅提供经过离线验证的准备模块，没有完成或启用视觉自主飞行。

历史 `HOST_VERIFICATION.json` 和既有发布标识保留，本轮证据单独引用。更新后的源码清单表示当前开发载荷；其 `source_commit` 是修改前基线、`working_tree_dirty=true` 表示验证的是该基线上的开发工作区，最终 Git 提交以仓库记录为准。
