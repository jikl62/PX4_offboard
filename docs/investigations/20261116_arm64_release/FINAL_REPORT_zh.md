# FC01 Stage3 预编译ARM64交付

本轮仅增加发行/安装工具和验证，不改变飞行控制算法。实际生成两个离线文件，源码/二进制身份逐文件绑定；不是把x86二进制改名为ARM64，也不要求板端重新编译。

## 发行身份

- 构建源码候选：`adc55d0b7c67e80bf9d77b1082208c898f95b756`，包含阶段3已通过的软件验证记录；生产控制逻辑仍为阶段2候选。
- 包内全部已提交源码基线：`56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a`，分支 `readability/fc01-chinese-20261111`。后续测试/报告提交不改变包内控制源码。
- 本轮新脚本/修复提交：254da94（打包/安装/服务）、416e7e9（大清单E2BIG）、5ff1255（systemd路径）、56424d7（新鲜状态读取顺序及安装启动等待）。没有修改main、推送、实机部署或PX4参数。
- 包：`FC01_OrangePi_20261116_stage3_arm64.tar.gz`，6497136字节（约6.2MiB），4974个普通文件，26个ELF全部AArch64。
- 包SHA256：`3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123`。
- 安装器SHA256：`7b35de4b7e498d46c56ac7b9abbf032c2ba64d1a76ab11177f9391bc4fc4e956`。

[完整交付摘要](evidence/DELIVERY.json)、[构建输入与安装产物摘要](evidence/ARM64_BUILD.json)、[ELF清单](evidence/ARM_ELF_INVENTORY.json)。控制器SHA256 `c446afae3d2a262bff132232cac77a40d2c9499ce8ea6a765d6d52c97f02cdc7`。

## 内容和依赖

包包含当前C++控制器/配置检查器、merged ROS2安装树、完整固定px4_msgs与FC01的C/C++/Python消息和typesupport、源码/配置、原运行管理脚本、视觉接口及新的预编译启动入口。Ubuntu系统/ROSHumble基础库复用目标机既有环境，不包含一整套Ubuntu/ROS离线安装盘。已有ROS不代表任意缺库可忽略：安装前逐ELF运行ldd、检查Python消息来源/typesupport和C++配置；缺依赖会在停旧前失败。安装不下载、不编译。

支持现有人工RC flight-test、domain0/network DDS、UART2 115200 Agent合同；不兼容配置明确拒绝，不暗中改成默认值。整份生效runtime继承，包括高度/RC/推力/其他参数，只改版本和两个路径元数据。视觉和自动Arm保持false。原Agent共享则保持身份；原监督器自有Agent若安全退出，仅允许按相同程序摘要/参数由原监督逻辑重建。没有安装器强杀或另建通信协议。

版本目录HOME/FC01_versions/20261116_stage3不可覆盖；原unit/dropins/environment files/runtime备份到HOME/FC01_navigation/prebuilt-backups。仅专用zzzz-fc01-prebuilt.conf切入口，保留原服务退出/重启策略。安装/回退要求新鲜未解锁接地、静止、零推力、RC安全非导航；不把旧JSON续鲜。真实源码差异见[边界摘要](evidence/SOURCE_BOUNDARY.json)，`git diff adc55d0 -- native/src native/config native/tools launcher.py standby.py navigation_service.py`为空。

## 验证与失败

1. ARM64 Ubuntu22.04用户态、GCC11 Release、ROS2Humble，QEMU6.2/PRoot5.4执行实际ARM编译器；2包colcon构建成功，31分钟。[完整构建日志](evidence/arm-colcon-build.log)。px4_msgs源码固定来自既有SDK的2.0.1源，不打包旧x86 SDK库。
2. 最终实际tar严格独立解包读回：全部文件集合/摘要/路径、链接/容量约束、ELF架构通过；安装树搬迁后26ELF/ldd/typesupport/nativeconfigcheck通过，[最终检查日志](evidence/arm-final-relocated-check.log)。真实systemd解析器还覆盖空格/%路径；SHA安装器固定相配tar摘要。
3. 宿主69项相关回归PASS，[原始输出](evidence/python-tests.log)：26旧服务、35安装器、6预编译服务、2发行证据。ARM虚拟机相同43项新相关测试PASS，仍是注入backend的事务单元测试，和下方真实systemd集成分开记录。
4. 真实完整ARM64VM安装验收见后续VM段；使用生产Backend/proc/systemctl/sudo/fuser/原监督器与真正ARM控制器。没有把fakebackend单元测试冒称系统安装。
5. QEMU-user跨进程FastDDS失败：直接ARM multicast socket选项ENOPROTOOPT，QEMU6.2/8.2均复现。单进程358/358与UDPunicast10/10通过，跨进程0；没有放宽控制校验掩盖。完整VM使用guestLinux内核，能实际DDS通信。[限制说明](evidence/USER_EMULATION_LIMITATIONS.md)、[原始失败归档](evidence/user-emulation-dds-failures.tar.gz)。初版夹具socket目录权限/停止时暂停反馈/图缺ACK等问题均保留，修正后宿主真实DDS夹具PASS：603批反馈、4552callback、198零目标、0命令、READY、正常退出0，见[归档](evidence/host-ground-harness.tar.gz)。

真实VM先后确认并修复：完整manifest单argv导致E2BIG、WorkingDirectory错误引用、慢systemctl消耗快照窗口、启动完整校验46.377秒超过30秒安装等待。前三项分别有真实失败与独立回归。新安装验收等待180秒仅属于安装事务；控制循环stall、反馈TTL、RC权限和飞行安全门均未修改。失败尝试/最终结果全部保存，不能只报成功。[独立入口审查](evidence/SERVICE_REVIEW_zh.md)、[安装审查](evidence/INSTALLER_REVIEW.md)、[最终小修复审查](evidence/FINAL_FIXES_REVIEW.md)、[打包器审查](evidence/BUILDER_REVIEW.md)。

## 现场范围

没有访问实际香橙派或PX4，没有实机解锁、启动电机或动力试飞。虚拟机的generic5.15内核/CPU和串口替身不能证明Zero3W的6.6内核、UART/XRCE或负载性能。原阶段3实机待验证项（估计重置/链路失联/真实高度收敛/下降窗口等）保持，发行不提供飞行安全证明。板端先拆桨未解锁安装/状态/服务核对，现场飞行由人员单独执行。

[直接安装、状态及回退命令](DEPLOY_zh.md)。完整安装在启动新服务后自动验收，不自动解锁或起飞；首次安装需要现场原服务及PX4/RC安全地面反馈，无法取得会拒绝。

## 真实ARM64/systemd完整安装验收

环境：QEMU-system-aarch64 virt/Cortex-A72、Ubuntu22.04 ARM64、generic5.15内核、systemd249真实PID1、ROS2Humble。普通orangepi用户通过sudo写unit，生产Backend读取真实/proc、systemctl、fuser与原Supervisor原子状态；实际C++控制器+真实DDS。PX4源永久未解锁/接地/静止，Agent只是持有PTY的无XRCE软件替身；不是实际Zero3W/PX4。

最终包`3c876f6f...d2a123`安装成功，服务工作目录与exe指向HOME/FC01_versions/20261116_stage3，[安装输出](evidence/vm/actual-install-7.log)。[安装后状态](evidence/vm/installed-final-state.json)只有一个控制器PID4623、READY_WAIT_ARM、未解锁/接地/静止、精确零推力、视觉false、自动armfalse。继承目标1.23m、RC导航档1，全配置只发生版本/源路径身份变化。

[重复安装](evidence/vm/actual-repeat-install-final.log)返回already_active=true；[重复后状态](evidence/vm/repeated-final-state.json)控制器PID仍4623，runtime不变、同一backup，不重启。期间外部Agent始终同PID671。

[手动回退](evidence/vm/actual-manual-rollback-final.log)恢复HOME/fc01-old入口，[回退后状态](evidence/vm/manual-rollback-final-state.json)唯一旧控制器PID5339，同样READY/安全地面/1.23m/RC档1。原runtime逐字节SHA929eb1cb029d3a6af38197e0d8e396ca9145a0d5b83a8465dd049f715cefe13c完全恢复；原unit未改、专用drop-in已移除。

失败自动回退实测为前一b4392e37包的30秒启动验收超时：[失败返回](evidence/vm/actual-install-6.log)、[真实服务/回退日志](evidence/vm/install6-automatic-rollback.log)。旧原生先优雅退出code0，新监督器尚在校验时被正常停服，再恢复旧exe/全部参数/DDS/地面并记录failed_restored。最终包改为180秒并成功安装；没有声称最终包又人为制造同样超时。最终源码对应的无节点超时、配置漂移、共享/ownedAgent交接失败等故障继续由确定性回归验证。

[汇总JSON](evidence/vm/FINAL_VM_RESULT.json)、[43项ARM单元输出](evidence/vm/final-arm64-unit-tests.log)、[guest原始归档](evidence/vm/guest-final-evidence.tar.gz)。实际重复安装/回退使用同一个最终SHA固定安装器，没有开测试绕过开关。测试源码/报告另行本地提交；交付包本身绑定56424d7，不因后续记录提交而变更。

剩余发行限制：两个重复Python生成库的RUNPATH文本仍含构建路径，实际支持入口通过相对overlay/包内lib选库，独立搬迁ldd/导入映射及VM均选中当前包/Humble/Ubuntu库，未选旧SDK。详见[最终包独立复核](evidence/FINAL_ARTIFACT_REVIEW.md)。不把“运行时未选旧路径”写成“二进制内完全没有任何构建路径字符串”。

## 复核与复现入口

[开发机与ARM验证命令说明](VERIFY_COMMANDS.md)、[VM实际命令](evidence/vm/COMMANDS_zh.md)、[完整VM结果](evidence/vm/RESULT_zh.md)。最终本地提交另包含修正后的隔离地面夹具和本目录证据；交付安装包保持上述56424d7源码身份，不为记录提交重复打包。原有非本轮未跟踪文件保留，未纳入发行提交。

生成安装器`--status`与包内`navigation.py status`实际补验退出0；从INSTALL_STATE读取的备份路径与首次成功回退一致，已回退后第二次请求正确拒绝覆盖（退出1）。见[原始命令输出](evidence/vm/user-status-and-repeat-rollback.log)。这些补验在同一VM磁盘重新引导后执行，不能将其PID/boot与首次安装事务混为一谈。原有monitor只保留使用说明，未在本轮另行声称完成交互监视验收。[交付文档独立复核](evidence/FINAL_REPORT_REVIEW.md)的两处文字建议已落实。
