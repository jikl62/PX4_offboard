# FC01 1.5 m 定高 Flight Test 候选版

本版本把运行入口分成两个显式模式：`diagnostic` 默认关闭所有动作权限；`flight-test` 必须由操作者同时选择 `--mode flight-test --confirm-flight-test`。Flight Test 保留 RC 手动解锁、Offboard 状态确认、RC 接管、Kill、Agent/反馈超时和 PX4 failsafe；服务启动不会自动解锁或自动起飞。

## 候选包和 ARM64 构建

在源码机生成包（包内为源码，不是预编译 ARM64 二进制）：

```bash
python3 build_navigation_bundle.py \
  ~/Desktop/FC01_OrangePi_Navigation_20261108_flight_test.sh \
  --directory-name FC01_OrangePi_Navigation_20261108_flight_test
sha256sum ~/Desktop/FC01_OrangePi_Navigation_20261108_flight_test.sh \
  > ~/Desktop/FC01_OrangePi_Navigation_20261108_flight_test.sh.sha256
```

香橙派必须已有 ROS 2 Humble、`px4_msgs` 的 `install_r632`、`cmake`、`g++`、PyYAML、`rclpy` 和离线 MicroXRCEAgent。不要覆盖旧目录：

```bash
NEW="$HOME/FC01_OrangePi_Navigation_20261108_flight_test"
FC01_NAVIGATION_INSTALL_DIR="$NEW" \
  bash ~/Desktop/FC01_OrangePi_Navigation_20261108_flight_test.sh --install-only

export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
bash "$NEW/navigation_boot.sh" build \
  --prefix "$NEW/install" --build-dir "$NEW/.build/arm64-flight-test"
```

`build` 只执行 CMake configure/build/install，不启动控制器、Agent 或 systemd。源码候选配置来自包内 `native/config`，不会静默读取旧工程的 `config/attitude_altitude.yaml`。

## Diagnostic 地面观察

默认模式或显式使用：

```bash
bash "$NEW/navigation_boot.sh" prepare-config --mode diagnostic \
  --file "$HOME/FC01_navigation/diagnostic-runtime.yaml"
bash "$NEW/navigation_boot.sh" check-config --mode diagnostic \
  --file "$HOME/FC01_navigation/diagnostic-runtime.yaml" \
  --prefix "$NEW/install"
```

Diagnostic 的 `flight_enabled`、`flight_actions_enabled`、`arm_enabled`、`takeoff_enabled`、`rc_navigation_enabled`、台架权限和视觉权限均为 `false`。用隔离 socket 启动 node 后，只观察反馈年龄、控制周期、回调预算、零推力和无控制命令；运行中的旧 `fc01_system_node` 必须先确认并保持唯一。

## Flight Test 配置

生成候选运行配置时必须显式确认：

```bash
bash "$NEW/navigation_boot.sh" prepare-config \
  --mode flight-test --confirm-flight-test \
  --file "$HOME/FC01_navigation/flight-test-runtime.yaml"
bash "$NEW/navigation_boot.sh" check-config \
  --mode flight-test --confirm-flight-test \
  --file "$HOME/FC01_navigation/flight-test-runtime.yaml" \
  --prefix "$NEW/install"
```

生成后的运行配置会使用 `aa_hover_thrust=0.50`、`aa_takeoff_reserve_thrust=0.04`、`aa_thrust_max=0.80`、目标高度 `1.5 m`，并保留 `arm_enabled=false`。RC 路线的动作门只在显式 Flight Test 模式下生成，实际起飞仍要求人工先在 Stabilized 中解锁，再在规定时间内切入配置的 Offboard 档位；程序不会在服务启动后自动解锁。

启动日志必须包含版本、模式、实际加载的配置路径、悬停推力、起飞余量、最大推力、目标高度、动作门和视觉状态；supervisor 状态还记录配置源路径。控制器的 phase/CSV 日志同时记录起飞调度目标、限幅后推力和最终发布推力。`0.54` 是离地前调度候选上限，不代表已证明足以离地；确认离地后进入高度闭环，理论全局上限仍为 `0.80`。

## 1.5 m 受控测试顺序

只有在拆桨 DDS/权限/反馈验收、唯一控制器检查、Kill/RC 接管检查、Agent 断开安全退出检查和 PX4 failsafe 核对均通过，并由现场操作者确认隔离区和应急操作后，才进入动力测试：

1. 飞机接地、螺旋桨和人员按现场规程隔离；CH7 Kill 正常，油门最低，CH6 在 Stabilized，CH5 未解锁。
2. 等待 `PRESTREAM`/`READY_WAIT_ARM`，确认日志中的 `flight-test`、参数和实际配置路径。
3. 人工在 Stabilized 解锁；在 RC 导航超时窗口内切入 Offboard。观察 `ARMED_GROUND` 后才会进入 `THRUST_RAMP`。
4. 观察离地证据、`ASCEND` 和 `HEIGHT_HOLD`，目标为相对起飞参考高度 `1.5 m`。NED 上升为负 `vz`，不得把 `z` 符号反转。
5. 人工发出降落请求或等待任务时限，确认 `DESCENT`、接地新鲜证据、解除 Offboard 和安全上锁。

任何无效四元数、NaN/Inf、估计器重置、反馈超时、Offboard 丢失、Kill、RC 接管或权限不一致都必须停止推进并进入既有安全流程。有效的 20.2°/90°姿态反馈不会单独触发旧 `EXCESS_TILT` 停机；这不等于允许以 90°姿态飞行。

## 当前边界

R2 离散安全事件现在按源独立锁存，异常后同一 `spin_some()` 中的正常样本不能覆盖下一控制周期将处理的异常；队列溢出闭锁为禁止输出。该修复已由离线 C++ 回归覆盖，但仍需在香橙派拆桨现场复核 DDS 行为。真实动力响应、推力标定和 PX4 实机采用不能由编译或离线测试替代。
