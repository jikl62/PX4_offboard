# FC01 1.5 m 定高功能恢复与最终交付

> **历史生命周期说明已替代。** 本文中的 `20261108.2`、关闭开机自启、旧重启策略及暂存目录换版命令仅代表当时版本。20261109 版本的独立部署、开机待命、飞后重新准备、异常恢复和回滚，以 [FC01 服务生命周期与独立升级说明](FC01_SERVICE_LIFECYCLE_20261109.md) 为准；不要继续使用本文的旧生命周期命令部署新版。本文保留原有定高控制与历史验证背景。

本轮基于 `a24bb249` 恢复、回归已有 RC 人工解锁定高流程，不重写控制器。入口版本 `20261108.2`；编译标识 `R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108`。保留 `aa_hover_thrust=0.50`、余量 `0.04`、最小 `0.12`、最大 `0.80`、slew `0.40/s`，高度 PID、姿态坐标系及视觉接口不变。

## 权限和原有流程

`flight_enabled=false` 正确且必须保留。`attitude_altitude` profile 的配置校验禁止打开这个旧位置/速度任务开关；RC 路线使用独立的 `flight_actions_enabled`、`takeoff_enabled`、参数确认、RC 接管确认与物理解锁/档位证据。`arm_enabled=false` 保留，程序不发送自动 Arm 或 RC 路线的 Offboard 进入命令。

实际流程为：

```
IDLE → RC_PRESTREAM → RC_READY_WAIT_ARM
→ RC_ARMED_WAIT_NAVIGATION → ARMED_GROUND → THRUST_RAMP
→ ASCEND → HEIGHT_HOLD → DESCENT → LANDED / REQUEST_DISARM
→ SAFE_DISARMED → RELEASE_OFFBOARD / 人工确认模式释放 → IDLE
```

操作者先在 Stabilized 人工解锁，然后在 8 s 窗口内切入实际 PX4 Offboard。slot 1 表示 PX4 `ManualControlSwitches.mode_slot=1`，不是 CH6 通道号；必须用现场反馈核对档位映射。`estimate_valid=false / xy_valid=false / heading_good=false` 不阻断本轮姿态定高路线；有效四元数、z/vz/z_deriv、源采样时间、有效标志、重置计数、必要状态和权限检查仍然保留。不宣称无水平定位时可以定点悬停。

Diagnostic 配置在 `native/config/diagnostic.yaml`，所有动作权限关闭。Flight Test 配置在 `native/config/flight_test.yaml`，必须显式选择 `--mode flight-test --confirm-flight-test`。运行参数取自本包 `native/config`，不会从 SDK 的旧 `config/attitude_altitude.yaml` 导入推力参数。未来视觉接口保留，两种模式均关闭视觉导航。

## 推力和安全路径

地面准备输出为零。推力爬升需求为：

```
0.50 × min(1, t/2 s) + 0.04 × clamp((t−2 s)/2 s, 0, 1)
```

离地前候选上限 `min(0.80, 0.50+0.04)=0.54`；保留独立离地高度、速度及新鲜 LandDetector 证据，时间或推力达到 0.54 不能自行证明离地。确认离地后高度/垂直速度闭环参与控制，上限按既有调度开放到全局 0.80，保留 slew、积分限幅和 anti-windup。不能据此认定 0.54 足以让实机离地；未满足证据会进入已有超时/安全路径。下降、接地与 Disarm 仍需独立的新鲜证据。

日志以 `thrust_planned_z`、`thrust_last_target_local_commit_z`、`target_local_commit` 区分计划值和实际本地发布；闭环另有 `planned_vertical_raw_thrust`、`planned_vertical_thrust_ceiling`（CSV 对应 `aa_raw_thrust`、`aa_thrust_ceiling`）。纯开环爬升的调度需求由上述公式核对，不把闭环无效字段当作有效需求。NED 向上为负 Z/vz，机体推力发布为负 Z 分量。

旧 `EXCESS_TILT` 自主中止保持移除，有效 20.2°/90°样本不会单独触发它。独立的 RC 地面入场姿态检查 `RC_GROUND_TILT_EXCEEDED` 和目标姿态限制保留，不授权以大倾角起飞。四元数无效、NaN/Inf、活动估计器重置、反馈超时、Kill、接管、Offboard 丢失等检查保留。PX4 固件及参数未修改。

本轮修复 R2 两个实际缺陷：队列溢出标志不再被后续赋值覆盖；在任何动力计划之前检查全部已锁存安全载荷，防止安全异常藏在同源良性事件之后。最新连续反馈与离散 FIFO 仍独立，不添加接收等待，不恢复旧的每条反馈强制控制周期。控制周期使用最新连续反馈，逐条检查独立队列中的安全载荷，不再把旧队头替换进带有新鲜时间戳的实时缓存；完成输出边界后才确认消费本批事件。异常的确认/清除以原有 Core 安全策略为准，后续正常样本不能清除活动安全锁存；未进入飞行的 RC 启动等待也不能因普通估计器尚未就绪而新增永久阻断。

## 香橙派更新：独立构建，不覆盖旧版

交付文件为 `FC01_OrangePi_Navigation_20261108_flight_test.sh` 及同名 `.sha256`；这是源码安装包。此次 C++ 安全修复使旧 ARM64 二进制不再足以证明已包含修复，需要一次针对本次源码的 ARM64 重编译，已通过的基础 DDS/频率/依赖验收无需重做。

旧入口对已有成功 `install` 可能直接跳过构建，不能在旧 prefix 上执行 build 后就声称更新成功。下面使用全新 staging 和真实新二进制；原有 SDK 仅提供 ROS/px4_msgs，不提供控制器或飞行参数。包默认不会覆盖不同的已有文件。

在开发机把 `dist/FC01_OrangePi_Navigation_20261108_flight_test.sh`、同名 `.sha256` 和 `FC01_20261108_final_validation.json` 复制到 U 盘。在香橙派文件管理器把三者复制到 `$HOME`，安全卸载 U 盘；无需联网或 SCP。随后在香橙派普通用户终端执行；安装包和校验文件假定放在 `$HOME`。如这些备份/暂存路径已存在，停止并改用新的唯一后缀，不覆盖它们。

```bash
set -e
cd "$HOME"
sha256sum -c FC01_OrangePi_Navigation_20261108_flight_test.sh.sha256
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
NEW="$HOME/FC01_OrangePi_Navigation_20261108_flight_test"
STAGE="${NEW}.incoming-20261108_2"
OLD_RELEASE="${NEW}.a24bb249-backup"
test ! -e "$STAGE"
test ! -e "$OLD_RELEASE"
FC01_NAVIGATION_INSTALL_DIR="$STAGE" \
  bash "$HOME/FC01_OrangePi_Navigation_20261108_flight_test.sh" --install-only
bash "$STAGE/navigation_build_fresh.sh"
cat "$STAGE/ARM64_BUILD_EVIDENCE.json"
bash "$STAGE/navigation_boot.sh" prepare-config \
  --mode flight-test --confirm-flight-test \
  --file "$HOME/FC01_navigation/final-runtime.yaml"
bash "$STAGE/navigation_boot.sh" check-config \
  --mode flight-test --confirm-flight-test \
  --file "$HOME/FC01_navigation/final-runtime.yaml" --prefix "$STAGE/install"
```

`navigation_build_fresh.sh` 拒绝 x86、已有 install/缓存/完成记录；按 `FC01_RELEASE.json` 验证源码哈希，使用已安装 ROS/px4_msgs，检查两份 ELF 的 AArch64 架构、动态依赖及本版标识。`ARM64_BUILD_EVIDENCE.json` 记录源码提交、二进制 SHA256、架构与依赖解析，现场必须保存它。上述步骤只构建、检查，不启动 Agent、控制器或动力。新版启动校验必须发现本次编译标识，旧 a24 二进制将被拒绝。

现场确认拆桨、接地、未解锁后，停止旧入口再换目录。保留已有 MicroXRCEAgent，不停止或重新安装它。若旧 standby 服务仍为 enabled，先人工 `sudo systemctl disable fc01-standby.service`，防止重启恢复第二个控制入口。

```bash
sudo systemctl stop fc01-navigation.service
test "$(systemctl show fc01-navigation.service -p MainPID --value)" = 0
if pgrep -af '[/]fc01_system_node([[:space:]]|$)'; then
  echo '还有控制器进程，保持停止并先核对来源'
  exit 1
fi
mv -T "$NEW" "$OLD_RELEASE"
mv -T "$STAGE" "$NEW"
cd "$NEW"
bash navigation_restore.sh configure --confirm-flight-test
```

`configure` 复用现有 UART2 Agent；拒绝第二个控制器、外部二进制、未知 service/drop-in 和缺少新鲜未解锁接地证据的情况。服务 unit 备份到日志打印的 `service-backups` 路径，设置 `Restart=no` 并关闭此试验入口的开机自启；不会自动 start。Unix Socket 由原有 supervisor 的独占锁和每次运行独立私有 IPC 目录处理，不要手动删运行中的 Socket。

成功必须出现：

```
FC01_NEW_SERVICE_CONFIGURED controller=...20261108_flight_test/install/lib/jikl_fc01_system/fc01_system_node
FC01_EXISTING_AGENT_RETAINED pid=...
FC01_SERVICE_NOT_STARTED boot_enabled=disabled ... rollback=...
```

操作者明确选择开始拆桨验收时才执行：

```bash
sudo systemctl start fc01-navigation.service
sudo journalctl -u fc01-navigation.service -n 100 --no-pager
bash "$NEW/navigation.sh" status
```

启动成功须同时确认服务 active、唯一控制器、`controller_build=R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108`、版本 `20261108.2`、`mode=flight-test`、新包配置源路径、实际 `$HOME/FC01_navigation/runtime.yaml`、hover 0.50、reserve 0.04、max 0.80、target 1.5、Arm=false/legacy flight=false/RC=true、visual=false，并看到 `RC_PRESTREAM / RC_READY_WAIT_ARM`（状态视图 `PRESTREAM / READY_WAIT_ARM`）。单有 active 不算验收通过。 可进一步用下面命令核对实际运行 inode 与本次 ARM64 构建哈希；它只读，不发送控制请求：

```bash
cd "$NEW"
/usr/bin/python3 - <<'PY'
import hashlib, json
from pathlib import Path
from launcher import process_list
entry=Path.cwd().resolve()
record=json.loads((entry/'ARM64_BUILD_EVIDENCE.json').read_text())
release=json.loads((entry/'FC01_RELEASE.json').read_text())
assert record['source_commit']==release['source_commit']
rows=[p for p in process_list() if Path(p.argv[0]).name=='fc01_system_node']
assert len(rows)==1, 'controller count must be exactly 1'
actual=Path('/proc',str(rows[0].pid),'exe')
assert actual.resolve()==(entry/'install/lib/jikl_fc01_system/fc01_system_node').resolve()
assert hashlib.sha256(actual.read_bytes()).hexdigest()==record['binaries']['fc01_system_node']['sha256']
print('FC01_RUNNING_BINARY_MATCHES_ARM64_BUILD',release['source_commit'],'pid='+str(rows[0].pid))
PY
```

原有 supervisor 仍会在进程终止后按既有策略重连，且仅在既有 Agent 消失后才可能自行创建 Agent；不要在试验中把此重连当作继续飞行授权。任何故障先人工停止本试验服务、保留日志，再确认安全地面状态；RC 人工解锁和新鲜物理状态确认始终必需。

## 停止和回滚

试验服务不是空中急停入口。动力试验中优先使用已验收的 RC 接管/Kill/PX4 失效保护；只有在现场确认接地未解锁后才进行版本/服务变更。正常地面停止：

```bash
sudo systemctl stop fc01-navigation.service
```

回滚先确认 MainPID=0、无其他控制器，再使用 configure 输出的实际备份路径；helper 验证 unit 哈希并再次要求新鲜接地证据，不覆盖未知改动：

```bash
bash "$NEW/navigation_restore.sh" rollback --backup "$HOME/FC01_navigation/service-backups/实际时间戳目录"
test ! -e "${NEW}.failed-20261108_2"
mv -T "$NEW" "${NEW}.failed-20261108_2"
mv -T "$OLD_RELEASE" "$NEW"
sudo systemctl daemon-reload
```

回滚保持服务停止/开机关闭，不自动启动旧版。若 configure 尚未修改 unit 就拒绝，只恢复目录即可，不伪造备份路径。

## 本轮回归和实机边界

新增 `test_rc_navigation production_1p5m` 使用当前 Flight Test 的推力、调度及高度参数，在 generic estimate/XY/heading 无效且 z/vz 有效、无视觉条件下，走完人工 Arm/Offboard→爬升→离地→1.5 m HEIGHT_HOLD→人工 LAND→新鲜接地/Disarm 确认→人工模式释放/IDLE。它使用连续合成 NED 轨迹验证软件逻辑，不是动力学仿真或真实飞行证据。独立权限和最终输出安全否决同时回归。

测试和提交的确切结果见随包外附 `FC01_20261108_final_validation.json`。原有未提交 `fc01_config_check.cpp` 用户改动保留，不纳入本次提交或发布源码。

已报告通过的旧版 ARM64 编译、DDS 接收、Diagnostic 和周期证据保留。本轮没有远程连接香橙派，也未执行实机解锁或起飞。尚需对本次二进制补充拆桨安全验收：同轮异常在真实 DDS 不丢失、RC 接管与 Kill、Agent 中断后的安全响应、实际 slot 1→Offboard 映射，以及本机 PX4 失效保护参数和无 XY 条件的退出方式。不得把用户状态单中 `ALL_PASSED` 当作新二进制这些项目的原始测量证据。

PX4 v1.16 官方说明要求持续 Offboard 存活信号；丢失后的动作由 `COM_OF_LOSS_T`/`COM_OBL_RC_ACT` 等参数决定。现场应记录这些实际值，并核对 `COM_FLTMODE1`、`COM_RC_OVERRIDE`、`COM_RCL_EXCEPT`、`NAV_RCL_ACT` 与当前无视觉/无 XY 条件兼容。没有读取或修改飞控参数，不能宣称 Agent 失联一定自动安全降落。

参考：[PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)、[PX4 v1.16 参数](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference)。

现场采集：`journalctl -u fc01-navigation.service -o short-iso`、`$HOME/FC01_navigation/{controller.log,standby.log,status.json,runtime.yaml}`，并保留 PX4 ULog/实际参数导出。每次记录包 SHA256、编译标识、模式映射、操作者及测试结果。0.54 能否实际离地、动力响应、飘移和场地隔离仍须受控实机确认；本轮不自动部署或触发动力测试。
