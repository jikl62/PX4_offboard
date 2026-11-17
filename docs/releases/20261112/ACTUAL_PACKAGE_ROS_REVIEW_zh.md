# 20261112 正式运行包：独立解包、依赖与 Humble 验收

本报告只针对已经实际解包的正式包，不是用工作区源码代替交付包做测试。

- 安装包：`FC01_OrangePi_Navigation_20261112_release.sh`
- 安装包 SHA-256：`f2726cfe18923c2e8783f19f81a536658dd9a99e28f9398b6ac05c5de5534927`
- 冻结源码提交：`32990e599a68fde377daadbb89710b77fda1fb82`；manifest 声明 `working_tree_dirty=false`。
- 实际解包目录：`/home/jikl62/.cache/fc01-release-20261112-8r2k_twx/actual-package/actual-release-home-ry9c6qhj/FC01_OrangePi_Navigation_20261112_release`。
- 载荷：95 项文件，1,543,406 字节（含清单）；清单覆盖 94 份其他文件。

## 验收结论

所有本子任务验收通过。实际解包目录的 95 个文件，在 fresh 构建、导入和只读 ROS 测试前后逐字节 SHA 相同，没有给载荷补文件或写入运行配置。94 份载荷另与冻结 Git 对象只读比较，全部一致；Git 对象只用于来源核对，没有用于构建、导入或通信测试的 fallback。

| 实际步骤 | 真实结果 | 耗时 |
| --- | --- | --- |
| 从实际包 `native/src` 全新 colcon Release 构建 | 1 个包，exit 0 | 61.982836 s |
| 两份 ELF 读取身份 | 2 / 2，x86_64；均未执行 | 静态检查 |
| 五种实际安装消息及 CDR | 5 / 5 类 | 0.529429 s |
| 包内模块导入 | 根 19 ＋ native tools 6 ＝ 25 / 25 | 0.578075 s |
| Python 语法 | 实际包 25 / 25 文件 | 同上 |
| Shell 语法 | 实际包 7 / 7 文件 | 静态检查 |
| C++ 本地 quoted include 闭包 | 缺少文件 0 | 静态检查 |
| 真实 ROS 视觉适配 | 7 / 7 检查 | 5.080919 s |
| 真实 ROS 目标速度只读诊断 | 11 / 11 检查 | 10.188893 s |
| 实际包被动采集生命周期 | 3 / 3 正常周期 | 5.087009 s |
| ROS 图前后检查 | 6 / 6 | 已计入通信阶段 |

以上是不同计量单位，不能相加成 unittest 方法数量。构建采用 `BUILD_TESTING=OFF`，构建成功本身不算 C++ 单元测试。

## 证明输入来自实际包

1. 每个步骤工作目录都是上述实际解包目录；CMakeCache 的 `CMAKE_HOME_DIRECTORY` 指向该目录内 `native/src/jikl_fc01_system`。
2. build/install/log 预先不存在，全部新建在本缓存，未复用此前验收的控制器或生成消息。
3. 五消息检查器显式把 MSG 源路径绑定到实际包；实际 ament prefix、Python 消息与类型支持必须位于本轮新安装前缀。
4. CMake 安装的 4 份视觉 Python 工具和 5 份 MSG，均与实际包相应文件逐字节一致。两份 ELF 仅由 `readelf` 读取；控制器中确有清单所写的既有 V5 标记。
5. 导入的根模块和六个 tools 全来自实际包。执行了纯配置计算和纯源码清单校验，确认目标 1.5 m、导航档位 1、程序解锁 false、视觉 false；没有调用服务入口、环境探针或硬件函数。
6. 验收驱动位于外部 cache，是只读测试工具；不会在实际包缺少源码或模块时从仓库补齐。

## 运行依赖闭包与排除决定

runtime-v1 正式包保留 19 个根 Python 模块、7 个根 Shell、2 个设置 JSON、完整 `native/src` / `native/config`、6 个正式 native tools 和 3 份交付/视觉说明。根模块中的 launcher、standby、autostart、fc01_control 作为真实导入依赖保留；不是只匹配 `navigation*.py`。

CMake 的相对 `../../tools` 安装路径可以在实际包目录结构中找到四个模块。每个生产 CPP/HPP 的本地 `jikl_fc01_system/...` include 对应文件存在。

预审发现的 `vision_replay.py`、旧 `native/scripts/install.sh` 与垂直回放工具依赖 native/tests；最终运行白名单已经明确排除这些历史开发入口，所以不是当前正式包遗漏。开发测试、审查证据、历史回放继续由完整仓库提供，不假称本运行包附带完整开发树。旧 README 也已排除，不再把旧版自动安装流程当新发行入口。

## ROS 通信与生命周期边界

使用仅本机发现的隔离域 223 / 224 / 225，清除 inherited discovery server / DDS profile 参数。每组测试前后图检查通过，仅见检查器自己的临时节点和 ROS 元数据 topic，未见真实生产端点。

视觉测试发布合成 Odometry / VisionHealth，检查转换、来源冲突撤销、UTF-8 字符串预算拒绝与新鲜配对、只读参数和 SIGINT 干净退出；速度诊断检查真实 payload、QoS、重复/乱序、来源/reset、失效/超时、退出快照与无控制发布权。

被动采集器来自实际包 `navigation_collect.py`；每轮只订阅 9 个观察 topic，当前隔离域没有数据，每轮结束 Context 均关闭。这三轮验证正常生命周期；没有把之前替身异常测试冒充本次真实 ROS 异常注入。

没有执行生产 `fc01_system_node` 或 `fc01_config_check`、Agent、UART、systemd、部署、解锁或电机程序。ROS 测试启动的只有合成测试源和只读适配/诊断节点。

## 尚不能据此宣称的内容

- 本机为 x86_64 Humble；不是 ARM64 原生构建、Orange Pi 性能或热降频验证。
- 使用现有外部 `px4_msgs` underlay：`/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs`，没有重建它；实机固件与消息版本匹配仍待现场确认。
- 没有相机/SLAM 精度、真实时钟同步、负载下时延、协方差完整变换或飞行闭环稳定性结论。
- 只读接口通信成功不授予视觉控制权；当前视觉仍关闭。
- 没有重新执行被排除开发树的全部单元测试，也没有把此包验收当作所有可能异常时序的证明。

## 证据入口

`verification.json` 为汇总；`candidate.json` 保存完整实际 manifest，`payload_before.json` / `payload_after.json` 保存全部载荷摘要，`frozen_git_object_identity.json` 保存冻结提交比较。

每阶段有 `*.command.sh`、`*.log`、`*.result.json` 和相应 `*.report.json`。`installed_identity.json`、两份 `*.readelf.log`、`actual_dependency_closure.json` 保留输入与安装来源；`driver_provenance.json` 标识外部验收驱动。build/install 缓存只是本机验证产物，没有打入正式源码包或部署设备。
