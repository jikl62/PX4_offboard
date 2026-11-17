# FC01 20261112.1 完整源码安装包交付

已生成可独立解包、供 Orange Pi Zero 3W 原生编译的新版源码安装包。开发机验收通过；尚未连接香橙派，不能把本次 x86 验证称为 ARM64 或飞行验收。

## 交付文件

| 文件 | 用途 |
| --- | --- |
| `dist/FC01_OrangePi_Navigation_20261112_release.sh` | 正式自解包源码安装包，767,820 字节，约 750 KiB |
| `dist/FC01_OrangePi_Navigation_20261112_release.sh.sha256` | 整包 SHA256 |
| `dist/FC01_OrangePi_Navigation_20261112_DEPLOY_zh.md` | U 盘复制、仅解包、ARM64 构建、配置、人工启动、状态与回滚说明 |

整包 SHA256：

```text
f2726cfe18923c2e8783f19f81a536658dd9a99e28f9398b6ac05c5de5534927
```

所有交付文件位于 `/home/jikl62/FC01_OrangePi_Navigation_20261028/dist/`。包内另含完整中文发行说明 `docs/FC01_ORANGEPI_RELEASE_20261112_zh.md`。

## 源码身份与本地历史

- 分支：`readability/fc01-chinese-20261111`。
- 本轮开始提交：`87fa7c15092d4d3638e32c0e4380dab01faf4cb2`。
- 已冻结的发行源码提交：`32990e599a68fde377daadbb89710b77fda1fb82`。
- 源码提交在工作区干净时生成清单，`working_tree_dirty=false`。
- 最后的归档提交包含本报告、根 `FC01_RELEASE.json`、安装包和校验文件；用 `git log -1 -- docs/releases/20261112/DELIVERY_zh.md` 可定位。最终完整 HEAD 另在交付消息给出。

先提交源码，再生成绑定该提交的清单和安装包，是为了避免把未提交修改冒充干净源码，也避免要求清单递归包含自身 Git 哈希。归档提交不改变包内源码：94 份载荷逐文件同时匹配源码提交、当前磁盘、包内 manifest 和实际解包结果。解包/构建之后逐项 SHA 仍一致。没有从旧 dist 复制载荷，也没有打包开发机 ELF。

版本 `20261112.1` 与控制器既有能力标记 `FC01_RECOVERY_DIAGNOSTIC_V5_20261110` 分别记录。后者保留用于能力检查；实际源码身份由提交和摘要证明，不能仅凭 V5 字符串判定新旧。

## 本轮修复与修改范围

原打包器递归收集整个 docs/tests/native，使新增历史归档与运行源码共同消耗 512 项、8 MiB 总量、1 MiB 单项限制。原安装测试实际复现 3 个 `Curated source payload exceeds extractor bounds` 错误。

现改为正式运行依赖白名单：28 个根文件、46 个 native/src 文件、10 个 YAML、6 个原生工具、3 份当前说明和 native/VERSION；加发布清单，共 95 项，解压总量 1,543,406 字节。保留完整当前原生源码树、五种消息、视觉接口和安装工具。全量 native/src 中的两份纯文本旧备份不参与 CMake；正式 CMake 显式编译当前两个 .cpp。

历史审查、测试、回放工具、构建缓存和配置备份不进入运行包；Git 仓库中的原文件和历史没有删除。依赖 native/tests 的历史脚本整体排除，避免保留缺少依赖的入口。两份视觉说明标明哪些示例属于仓库开发工具；旧版导航 README 不进入本次包，避免混淆旧安装流程和当前垂向诊断策略。

自解包器 HEADER 与基线逐字节一致。三项容量限制、路径/链接拒绝、manifest 摘要、归档 SHA、整包 SHA、CRC、原子发布独立目录和拒绝覆盖均保留；旧清单不静默刷新。当前新 manifest 是显式生成的。

源代码提交的 13 份修改如下；后续归档仅增加清单、交付物和本目录证据：

| 文件 | 修改原因 |
| --- | --- |
| build_navigation_bundle.py | 运行依赖白名单、当前发行身份、继续校验原预算 |
| navigation_service.py | 仅 ENTRY_VERSION：20261110.7 → 20261112.1 |
| docs/FC01_ORANGEPI_RELEASE_20261112_zh.md | 完整部署说明 |
| docs/VISION_NAVIGATION.md、docs/VELOCITY_COMMAND_INTERFACE.md | 标明运行包与仓库开发工具的范围 |
| tests/test_release_payload_policy.py | 新增 8 项独立依赖/排除/预算/清单/链接回归 |
| tests/test_navigation_install.py、tests/test_vertical_diagnostic_bundle.py | 当前发行范围和真实树提取回归；保留安全拒绝 |
| tests/test_navigation_service.py、tests/test_navigation_deploy.py | 当前发行身份断言 |
| tests/test_overnight_service_readability.py | 精确允许两个已审核的版本展示差异，其余 AST/调用严格比较 |
| tests/test_stage2_python_equivalence.py、tests/stage2_python_trace.py | 保留原始轨迹；额外仅固定 ENTRY_VERSION，严格比较包括 SHA 的完整旧/新行为 |

## 真实测试结果

| 项目 | 本轮结果 | 证据 |
| --- | --- | --- |
| 原先报错的两套安装测试 | 原 33 项全部通过，3 个容量 ERROR 消失 | packaging_regression.json、logs/packaging-targeted-final.log |
| 新载荷策略 | 新增 8 项通过；分别验证 builder/extractor 的三项原限制仍拒绝越界 | tests/test_release_payload_policy.py、packaging_regression.json |
| 根 Python 全量 | **643/643 方法通过**，109.118 秒，无失败/错误/跳过 | logs/python-full-final.log |
| 原生辅助 Python | **41/41 方法通过**，23.474 秒，无失败/错误/跳过 | logs/native-python.log |
| 针对性安装/身份/历史差分 | 86 个不同方法通过，是根 643 项的子集，不重复相加 | packaging_regression.json |
| 实际最终包独立检查 | CRC、95 项、86 项必要文件集合、逐文件 Git/SHA、权限、install-only、复用、修改冲突/链接/整包损坏拒绝通过 | ACTUAL_PACKAGE_VERIFICATION.json、verify_actual_package.py |
| 实际包全新 x86 Humble 构建 | 成功，61.983 秒；CMake 输入是实际解包目录，未重编 px4_msgs | ACTUAL_PACKAGE_ROS_VERIFICATION.json、logs/actual-package-host-build.log |
| 消息及安装树 | 5 种消息生成/CDR、4 个安装视觉模块与原源码字节匹配 | interfaces.json、installed-identity.json |
| 模块与语法 | 25 个模块导入、25 份 Python、7 份 Shell 通过；两份部署说明 24 段 Bash 语法通过 | imports.json、INDEPENDENT_BUNDLE_REVIEW.json |
| 真实只读 ROS 通信 | 视觉 7 项、速度诊断 11 项、正常采集退出 3 轮、ROS 图前后 6 项通过 | ACTUAL_PACKAGE_ROS_VERIFICATION.json、vision-smoke.json、velocity-smoke.json |
| 原生 ELF | 两份 x86_64 ELF 只读结构/能力标记检查通过，未执行 | ACTUAL_PACKAGE_ROS_VERIFICATION.json |
| 实际 ARM64 入口 | 在开发机加载既有 Humble/SDK 后，正确拒绝 x86，未创建 ARM64 构建产物；实际包 source gate 通过 | host-architecture-rejection.json、actual-build-source-gate.json |
| 可重复打包 | 全量回归之后重新生成，整包逐字节相同 | reproducibility.json |

不同验证单位不能混称为“单元测试总数”。本轮只按 643+41=684 个 unittest 方法报告；消息、通信检查和语法检查另列。

保留原失败证据：最初 3 个容量错误在 logs/original-*-capacity-*.log；发行元数据改变后，第一次全量回归因旧差分断言出现 11 个 FAIL（含子测试），在 logs/python-full-before-metadata-test-update.log。修正仅针对具体版本常量：保留旧/新原始轨迹，另跑只固定该常量的当前实现，完整轨迹和备份 SHA 严格等于旧实现；没有放弃权限、路径、异常和时序比较。最终全量回归通过。原始日志保留字节，少量夹具输出不是 UTF-8，不影响末尾 unittest 汇总。CMake 原始日志的行尾空格也保留；源码/文档的 diff 检查排除原始日志，没有为格式检查重写取证输出。

独立验收脚本最初把 native/src 中已跟踪的纯文本备份也判作不应出现的文件，随后依据“完整原生树、CMake 显式当前输入、无 ELF”纠正了验收假设；未因此改动控制器或放宽解包限制。

## 飞行行为与配置保持

基线 59 份 native/src 和 native/config 文件逐字节相同。服务完整 AST 在只还原 ENTRY_VERSION 后相同。未修改高度、姿态、推力、RC 接管、Kill、Failsafe、锁存、地面参考、降落、上锁、任务重置或最终发布顺序。C1～C4 等已有候选仍隔离，未夹入发行源码。

| 配置 | 保持值 |
| --- | --- |
| 人工 RC 解锁 | 保留；程序 arm_enabled=false |
| rc_target_height | 1.5 m |
| aa_hover_thrust | 0.50 |
| aa_takeoff_reserve_thrust | 0.04 |
| aa_thrust_min | 0.12 |
| aa_thrust_max | 0.80 |
| aa_thrust_slew | 0.40 |
| visual_navigation_enabled | false |

这证明本次发行没有改动控制逻辑，不证明现有控制逻辑在所有硬件/时序下都正确。

## 目标板使用边界

按照 dist 中文说明：校验 → `--install-only` → 现有 SDK 的 `offline` → 停止旧服务 → `configure` → 人工 `systemctl start` → status/report。安装到独立目录，不覆盖旧安装。configure 要求已存在的匹配 Agent，不会替用户停服务或启动 Agent；完整 activate 会启动服务，说明中已区分。回滚使用实际打印的备份，成功后保持服务停止。

未提供当前可连接的目标板会话，本轮未做 ARM64 原生构建、板端基础启动、资源负载、UART/DDS/PX4 实际反馈和动力试验。外部 SDK 的 px4_msgs 必须与实际固件匹配；开发机成功不能代替这项现场验证。新包保留 ARM64 全新构建和严格 ELF/源码/依赖证据入口，目标机按 offline 执行后才生成 ARM64 证据。本包没有预置/冒充 AArch64 二进制。

当前正式包没有发现缺少运行依赖、提取或 x86 构建的阻断项。非阻断维护边界：白名单未来新增依赖必须同步维护；当前实际依赖完整性已有独立验证，但合成的微型源树仍可让 builder 跳过不存在的可选条目。

本轮没有 GitHub 推送、main 修改、PR/合并、设备部署、生产控制节点、Agent、服务启动、解锁或电机操作。完成本地归档后停止发行任务。
