# 阶段 3：可执行验证命令

这些命令不会解锁、切模式、启动控制器或更改 PX4 参数。本阶段修复尚未发行或部署；板端现有 20261112.1 不能冒充本轮源码。先在开发机复跑，再由独立发行流程制作新版本、ARM64 编译并核验身份。

## 开发机离线复跑

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
python3 docs/investigations/20261116_stage2/run_regression.py \
  --output "$HOME/.cache/fc01-stage3-recheck-$(date +%Y%m%d_%H%M%S)"
```

退出 0 才表示全部项目符合预期。`repro-baseline` 的退出 **1 是已知缺陷的预期复现**，编译失败不算复现成功；`repro-current` 必须退出 0。脚本记录源码前后 SHA、全部命令/退出码、未归一化原始差分和仅去除新增诊断的差分。这里只验证合成反馈，不证明动力学稳定。

只跑本次离地修复：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include -I native/tests \
  native/tests/test_20261116_takeoff_expiry.cpp -o /tmp/fc01-stage3-expiry
/tmp/fc01-stage3-expiry
```

ROS Humble 独立构建（只构建，不启动）：

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
FC01_STAGE3_BUILD="$HOME/.cache/fc01-stage3-ros-$(date +%Y%m%d_%H%M%S)"
export FC01_STAGE3_BUILD
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH bash <<'BUILD'
set -euo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
colcon --log-base "$FC01_STAGE3_BUILD/log" build --base-paths native/src \
  --packages-select jikl_fc01_system --build-base "$FC01_STAGE3_BUILD/build" \
  --install-base "$FC01_STAGE3_BUILD/install" --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
BUILD
```

## 香橙派现有运行身份：只读取证

从本地显示器/网络终端执行，保存目录会写到桌面；不重启服务，不操作串口。

```bash
FC01_STAGE3_EVIDENCE="$HOME/Desktop/FC01_stage3_$(date +%Y%m%d_%H%M%S)"
mkdir -m 700 "$FC01_STAGE3_EVIDENCE"
systemctl show fc01-navigation.service \
  -p MainPID -p ActiveState -p SubState -p ExecStart > "$FC01_STAGE3_EVIDENCE/service-show.txt"
systemctl cat fc01-navigation.service > "$FC01_STAGE3_EVIDENCE/service-unit.txt"
cat /proc/sys/kernel/random/boot_id > "$FC01_STAGE3_EVIDENCE/boot-id.txt"
uname -a > "$FC01_STAGE3_EVIDENCE/uname.txt"
ps -eo pid,ppid,lstart,args > "$FC01_STAGE3_EVIDENCE/processes.txt"
for name in runtime.yaml status.json controller.log controller.log.1 controller.log.2 standby.log; do
  if [ -f "$HOME/FC01_navigation/$name" ]; then
    cp -p "$HOME/FC01_navigation/$name" "$FC01_STAGE3_EVIDENCE/$name"
  fi
done
sudo journalctl -u fc01-navigation.service --since '30 min ago' --no-pager \
  > "$FC01_STAGE3_EVIDENCE/journal.txt"
for pid in $(pgrep -x fc01_system_nod); do
  sudo readlink -f "/proc/$pid/exe" > "$FC01_STAGE3_EVIDENCE/exe-$pid.txt"
  sudo sha256sum "/proc/$pid/exe" > "$FC01_STAGE3_EVIDENCE/exe-$pid.sha256"
done
python3 -m zipfile -c "$FC01_STAGE3_EVIDENCE.zip" "$FC01_STAGE3_EVIDENCE"
sha256sum "$FC01_STAGE3_EVIDENCE.zip"
```

Linux `comm` 的 15 字符截断名为 `fc01_system_nod`；如查不到 PID，直接从 processes.txt 中核对，**不能随便对其他 PID 哈希并声称是控制器**。拷贝运行中轮转日志并非原子快照，可能跨边界；同次验证还应从 READY 前持续保存输出。原熟悉状态入口可用：

```bash
python3 "$HOME/FC01_OrangePi_Navigation_20261112_release/navigation.py" status
python3 "$HOME/FC01_OrangePi_Navigation_20261112_release/navigation.py" report
```

入口路径仅适用于阶段 1 已采集版本；以 service-unit.txt / processes.txt 的实际路径为准。

## 新版无动力板端准备

本轮没有生成/更新发行身份，**不要直接在旧发行目录覆盖几个文件**。下一阶段先基于本轮提交制作独立新安装目录，保留旧目录和服务回滚；经发行清单核验后，在新目录运行既有入口：

```bash
# 新版完整发行目录中，显式使用既有ARM64 SDK；只编译，入口不启动服务。
bash ./navigation_build_fresh.sh
file ./install/lib/jikl_fc01_system/fc01_system_node
sha256sum ./install/lib/jikl_fc01_system/fc01_system_node
```

`file` 必须显示 AArch64，构建证据必须绑定本轮源码；本轮开发机 x86 ELF 不能拷去执行。这里未提供自动启动/动力试飞命令。

## 同一次试验必须保留的证据和判据

- MTF-02P 已直连 PX4、Position 稳定是现有硬件基线，复用原参数、串口和解锁检查；不重新调参。补取**同配置真实稳定 Position ULog**。
- 最新版异常任务的 companion 日志从 READY 前开始，配同次 PX4 ULog、boot/PID、运行二进制哈希、runtime、源码发行身份。ULog 时间和主机 monotonic/ROS 同步时间要通过可核实事件对齐，不能直接相减。
- 核对 z/vz/z_deriv 与有效位/reset、地面 z0 和 `z0-z`，本次有效 rc_target_height；Ramp 的提前闭环、独立离地证据、Ascend 的轨迹/误差/速度/持续稳定窗口。旧运动超过 land_timeout 不能被新 detector 重新拼接。
- `range_*` 只核对最近完成的 200ms 区间：sample_stamp 是最新样本，区间比较可保留到下个区间。必须为有效新鲜的 PX4 dist_bottom 且 RANGE 位存在；它不是已认证的 MTF 原始测量，也不保证独立于 EKF z。不可用不阻断原起飞。
- `rc_queued_safety_mask` 位0..9：Kill、failsafe、校准、HIL、preflight未过、电池未就绪、rc_live失败、轴不安全、摇杆移动、辅助开关不安全。可同时置多位；source 枚举0..6为Attitude/Vertical/Status/Failsafe/Authority/Battery/Land。源水位是重建事件快照中的各源戳，不代表跨源全局到达顺序。
- `abs(vz-z_deriv)` 单独持续偏差仍只告警；NaN、reset、位置/导数积分不符、过期、接管、Kill和部分发布仍应拒绝。不得为了测试继续发布旧动力或补心跳。
- 如发生 reset/心跳失败，在同次 PX4 ULog 中核对 q、delta_q_reset、counter、最后实际收到的 OffboardControlMode、mode/failsafe 与 COM_OF_LOSS_T/COM_OBL_RC_ACT；本地 sink 返回不等于 PX4 收到。按时间先后评估 No offboard signal，不能倒推唯一原因。

板端 ARM64、新版真实 DDS 通信、实际测距性能、飞行收敛及 PX4 失联退出延迟在本轮均未实测。TSan 未执行，不声称通过。
