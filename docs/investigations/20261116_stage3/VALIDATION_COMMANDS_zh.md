# 阶段3复跑、现场核对与回退

## 开发机：真实 ROS 入口的合成测试

这是隔离 domain191/localhost 下的生产 C++ 节点与合成 PX4 peer。**只在不连接 Agent/串口/PX4 的开发机运行**；脚本不是板端试飞入口。它会在合成系统中产生非零姿态/推力消息，但没有电机执行器。不要把它接入现场 DDS 域。

已核验可复用 ELF：

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
  bash docs/investigations/20261116_stage3/run_ros_matrix.sh \
  /home/jikl62/.cache/fc01-stage2-20261116/ros/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node \
  "$HOME/.cache/fc01-stage3-repeat-$(date +%Y%m%d_%H%M%S)"
```

夹具运行前核对阶段2 ROS_BUILD.json 中 ELF 与47项源码摘要；输入变化必须拒绝，不能冒认旧ELF。最终运行使用同一来源输入和二进制，本阶段无生产改动。若构建缓存不存在，按下段独立构建，核对新ELF来源；此夹具刻意固定已核验产物的哈希，新构建不同哈希时需先完成新构建证据审核，不能盲改期望值。

```bash
FC01_S3_BUILD="$HOME/.cache/fc01-stage3-build-$(date +%Y%m%d_%H%M%S)"
export FC01_S3_BUILD
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH bash <<'BUILD'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
colcon --log-base "$FC01_S3_BUILD/log" build --base-paths native/src \
  --packages-select jikl_fc01_system --build-base "$FC01_S3_BUILD/build" \
  --install-base "$FC01_S3_BUILD/install" --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
BUILD
```

现有缺陷回归、状态机差分与 ASan/UBSan：

```bash
python3 docs/investigations/20261116_stage2/run_regression.py \
  --output "$HOME/.cache/fc01-stage3-offline-$(date +%Y%m%d_%H%M%S)"
```

基线缺陷复现的退出1是预期；编译失败不算复现。执行器总退出必须0。

## 现场：仅拆桨、保持未解锁

本阶段未连接香橙派。本轮不提供解锁、起飞或带桨命令。复用现有已通过的串口、PX4参数和Position配置，不改变MTF直连关系。新候选须先走独立发行/ARM64构建核验流程，不往旧发行目录覆盖几个源文件；旧20261112.1安装包不含阶段2修复。

1. 拆下全部桨叶，确认PX4处于DISARMED；遥控器保持解锁开关OFF、非导航档、低油门和安全中杆。此次不拨动解锁/导航触发。
2. 用既有状态入口和 `systemctl show/cat` 核对实际服务、入口目录、运行ELF、runtime、源码发行清单。此前身份取证命令见 [阶段2现场取证](../20261116_stage2/STAGE3_COMMANDS_zh.md)。它们只读服务，不重启服务。未知路径以实际 ExecStart 和进程命令行为准。
3. 对独立新版源码只执行构建入口 `bash ./navigation_build_fresh.sh`，核对 `file ./install/lib/jikl_fc01_system/fc01_system_node` 为 AArch64，并保存构建证据/源码摘要。**当前Git树的历史 FC01_RELEASE.json 未在本轮重签，不能冒充新版发行**，不能直接拿x86 ELF到板上运行。
4. 在经过独立发行核验、仍未解锁的前提下观察：RC_READY_WAIT_ARM、armed=false、z0稳定、relative_height≈0、目标1.5m、thrust_body全零、视觉false、源年龄/reset不跳变。保存READY前后至少30秒同期日志，记录负载与最大循环周期。此步骤未在本轮实测；服务切换由现场人员在有备份的维护窗口执行。
5. 不在当前正在控制真实飞控的DDS域发布本夹具的合成消息，不人为断开正在控制飞控的反馈。真实RC/Kill/失联时延与动力学由现场人员另行组织，不能把此阶段模拟结果当作放行。

后续现场需保存同一次PX4 ULog + companion/controller.log + runtime + service入口 + boot_id/PID + ELF SHA。对齐ACTIVE_QUATERNION_RESET、阶段变化、PX4最后收到的Offboard心跳与No offboard signal；时间域不能直接相减。补充真实稳定Position ULog用于同配置比较。测量超调、稳定窗、保持误差、XY位移和PX4实收最大心跳间隔；模型的XY=0不作真实漂移基准。

## 回退

本阶段无生产改动、无部署。候选生产代码仍为阶段2 `e8652ad686c52502ea877fd23b5805d6b64e0da6`，完整基线文档HEAD为 `8767608f79a56e346d5654228770f948841fe0e3`。要取得阶段2独立源副本可用只读导出，不触碰当前工作区：

```bash
git archive --format=tar.gz \
  -o "$HOME/FC01_stage2_8767608_source.tar.gz" \
  8767608f79a56e346d5654228770f948841fe0e3
```

该文件是源码归档，**不是已签发行安装包**。本阶段新提交只有测试/报告，撤销它们不会改变飞控行为。若人工审核决定撤销阶段2离地期限修复，单独审查 `git revert 63fde0293e61de4d85252f8bd19cf3c684e37067`；诊断提交为 `e8652ad...`，回退后必须重测，不能把撤回已确认修复当作更安全。此处未执行revert/reset/checkout。

现场回退沿用独立旧安装目录、原服务备份及既有 `navigation_restore.sh`，先核实其帮助/目标和服务备份；本轮没有替换unit、路径或正在运行的程序，因此当前设备不需要因本轮测试而回退。维护时保持拆桨未解锁，不盲目恢复未知版本或PX4参数。
