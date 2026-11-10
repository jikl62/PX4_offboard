# FC01 离线控制运行稳定性：源码审计与最小修复

日期：2026-11-07。工作分支：`dev/offline-control-stability-20261107`。本轮范围为伴随计算机控制软件；不部署、不改飞控参数、不刷固件、不执行解锁或动力试验，不重复 UART 收发与 DDS 基础连通性测试。

## 1. 修复前审计结论

有两处能够直接由源码确认的时序问题，也有尚不能确定原因的现场现象。以下结论先于修复验收，不能用构建成功代替实机结论。

| 项目 | 已取得的证据 | 结论与处理范围 |
| --- | --- | --- |
| 发布边界的计时点过早 | 基线 `boundary_context()` 在 `snapshot()` 之前采样 `steady()`，随后使用这个旧时间计算控制有效性；`snapshot()` 内含多次 DDS graph 查询 | 能确认 snapshot 自身耗时未计入当次边界预算。修复应在 snapshot 完成后重新采样单调时间；保留最终安全复核 |
| 当前轮回调未计入发布预算 | 基线在 `spin_some()` 之后才由 `tick()` 设置 `control_started_`；上一轮诊断不能及时阻断本轮首次长回调 | 能确认当前回调耗时存在检查盲区。预算应从回调执行前开始，覆盖回调、控制和发布前检查 |
| 每轮临时 executor | 基线主循环调用便利函数 `rclcpp::spin_some(node)`；该便利路径创建临时 executor，并添加/移除节点 | 能确认存在可避免的生命周期工作。改为单个持久的单线程 executor，保持回调后执行 tick 的顺序。不能据此断言它造成了 336 ms 停顿 |
| 多次 graph 查询 | 基线每次 snapshot 都查询所有权、反馈发布者和控制订阅者；发布边界又多次获取 snapshot | 能确认重复查询存在，尚无现场分段耗时证明它是瓶颈。本轮先集中查询代码并记录次数和耗时，每次 snapshot 仍实时查询，不引入拓扑缓存 |
| 源时间年龄约 0.489 s | 已有报告中 `local_age_s≈0.000018`，但 `local_source_age_s≈0.488878`、`local_sample_age_s≈0.49072`；姿态源年龄约 0.48392 s，阈值为 0.5 s | 接收新鲜与源样本新鲜是不同证据。真实源延迟、DDS 时间映射偏移、回调排队和主机时钟变化均尚未被排除；不能认定 NTP 或 graph 是根因，也不能扩大阈值 |
| 历史 336 ms 控制周期 | 报告出现 `max_cycle_period_s≈0.336`，现行阈值为 0.3 s | 确认曾记录过超限周期，不能从原有总量指标分辨回调、IPC、graph、日志或调度耗时。需要本轮增加的分段证据 |
| 严重故障锁存 | `boundary_fault_latched_` 阻止重新准备；源戳倒退由 `Stamp`/样本进展跟踪永久记忆；部分发布错误进入 SAFETY | 这些保护不能普遍自动清除。只研究尚未出现解锁意图、从未发生部分发布的地面准备阶段暂时失鲜恢复 |

原控制核心已经具备跨 step 周期间隔超限保护；本轮补齐的是适配层当前执行预算，不能解读为旧系统没有 stall 保护。

现场尚未确认：修改后 ARM 平台实际周期上界、源时间差的成因、PX4 采用设定值、真实离地或自主飞行。上述内容仍为“待实机确认”。

## 2. 安装包、源码与当前工作区的对应关系

本地存在 `dist/FC01_takeoff_readiness_20261103_source.tar.gz`，其 SHA256 为：

```text
0b6a246ee3c557c79822ad1304fe56ea7368743d192e5f6a42f61fe5b56141bf
```

该值与 `dist/FC01_takeoff_20261103_RELEASE.json` 一致。发布清单记录基线提交为 `6c5b185526b7ac194138d05ec9698fccadf288c0`，分支为 `fix/takeoff-readiness-20261103`。修复开发前工作区 HEAD 为 `fcc35162536da75a00e0ae995e8ef50cf2244d96`；这段提交差异为 UART 审计工具、测试与文档，未修改本轮涉及的原生时间校验、控制运行时、发布边界、配置生成和 UI 行为。

逐文件比对本地源代码归档与修复前工作区：控制源码与归档一致；差异限于 README、既有通信/起飞审计文档和用户在 `fc01_config_check.cpp` 中的缩进/空白编辑。用户编辑应保留，不作为本轮修复提交内容。

发布清单记录安装器 `FC01_OrangePi_Navigation_20261103_takeoff.sh` 的 SHA256 为 `0b42777100ad5f727f3c9fba1b1d7c272c9bc21790fda3344c33668b6cc87408`，与用户交接材料一致，但本地没有该安装器原文件。因而本轮能够独立核对的是源代码归档和发布清单，不能独立证明香橙派当前二进制、安装器内所有文件和 `runtime.yaml` 的实际字节一致。用户提供的现场安装信息与交接审计是部署版本关联证据；实际运行文件哈希仍需在需要验收时只读留存。

清单中的“182 项 Python、60 项 C++、x86 ROS 2 Humble 构建通过”是 20261103 的历史结果，不是本轮修改后的测试结果；它也未声称 ARM 构建或飞行验收完成。

## 3. 两条故障链与实际源码入口

两条历史错误必须独立分析：

1. `RC_PRESTREAM` 中出现 `publication_blocked` → `PUBLISHER_BOUNDARY_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE`。含义为该次发布边界未满足源时钟/样本年龄要求，不能只由错误字符串判定到底哪一路数据失效。
2. `PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL`。含义为姿态目标可能已经进入本地发布 sink，但在后续最终复核时控制预算失效，Offboard 心跳被阻断；必须保留部分发布故障处理。

主要调用关系如下，原交接中的行号对应 20261103 包，修改后应按函数名定位：

| 路径/函数 | 作用 |
| --- | --- |
| `source_clock.hpp` 的 `source_stamp_in_clock()`、`source_sample_in_clock()`、`SourceSampleProgress` | 分别校验信封时间、原始样本时间和样本实际推进；新信封不能为冻结样本续命 |
| `runtime.hpp` 的 `Stamp::accept()` | 记录接收单调时间，拒绝不前进或倒退的信封，保留倒退证据 |
| `fc01_system_node.cpp` 的订阅回调、`snapshot()` | 接收反馈，计算接收/源时间有效性、RC 状态及端点所有权 |
| `Runtime::step()`、`Core`、`RcNavigationStart` | 飞行阶段与准备状态机、控制计算、RC 权限判断 |
| `SystemNode::boundary_context()` | 发布前重新获取反馈与唯一控制源证据，校验本轮执行预算 |
| `publication_boundary.hpp` 的 `publish_attitude_resampled()` | 三次 context：初次准入、打包后目标发布前、目标发布后心跳发布前 |
| `SystemNode::refuse_publication()`、异常捕获 | 明确记录拒绝、阻断输出；部分发布及严重故障保持锁存 |
| `Runtime::reprepare_rc()` | 仅在既有安全恢复条件允许时重新准备，不绕过 publication fault |
| `main()` | 单线程执行订阅回调、tick、周期观测与剩余周期等待 |

`source_clock_validated=true` 表示程序完成了校验，不能当作 PX4 timesync 已收敛的独立证据。三次边界复核均须保留；允许发布只证明本地执行路径通过，不能证明 PX4 收到或采用。

PX4 对 Offboard 持续心跳和失效退出的约定见 [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)。规范用于核对设计约束，不作为现场故障根因证据。

UI 在当前反馈失鲜时保留 WAITING，用于避免把旧进程的 SAFETY 当成新进程当前状态。已有单元测试明确约束这一行为，本轮不改变它。

## 4. 完全离线的时间约定

本机 TTL、周期、等待窗口和耗时使用 `steady_clock`。跨机源样本年龄仍使用 ROS/DDS 所约定的主机时间域与 PX4 uXRCE-DDS 的时间映射；不能直接把 PX4 原始启动时间与 Linux `steady_clock` 相减。稳定但日历日期不准确的 Linux 时钟不会因其日期本身失效；是否可用取决于跨机映射和源样本年龄，而非 Wi-Fi 或 NTP 状态。

本轮不要求联网、不主动校时、不关闭源校验、不凭接收时间替换源时间、不向源戳添加估算偏移。保留 0.5 s 源年龄约束、现有未来时间容差和 0.3 s 控制停顿阈值。真实失步、PX4 重置、源戳倒退或控制停顿仍应拒绝输出。

新增主机时钟诊断比较“系统时钟增量”与“单调时钟增量”，不相减两者绝对 epoch。误差既可能包含系统时钟变化，也可能包含两次采样之间的调度延迟；它是疑似跳变线索，不能单独证明系统时钟跳变，也不补偿年龄或授予控制权限。缺失/不可用的年龄和中间件接收时间显示为 `null`，不得用零冒充有效测量。

时间模型的规范核对：[PX4 v1.16 ROS 2 user guide](https://docs.px4.io/v1.16/en/ros2/user_guide) 说明 uXRCE-DDS 的时间同步关系；[ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html) 区分 System、ROS 与 Steady 时间。它们不能替代当前固件、Agent 和主机运行时的实际时间证据。

## 5. 最小修复与诊断结构

以下修改已在开发分支实现。构建与离线回归结果见第 7 节，现场表现仍待验收。

| 修改 | 目的与限制 |
| --- | --- |
| 持久的 `SingleThreadedExecutor` | 避免每轮临时 executor；保持单线程回调→控制顺序，不并发共享状态 |
| `CurrentControlCycle` | 在回调前开始预算；snapshot、控制和最终发布检查使用新采样单调时间；本轮过期不能靠再次调用校验恢复 |
| snapshot 先查询 graph，再采样接收/源年龄 | 防止 graph 查询耗时被旧计时点掩盖；没有拓扑缓存，每次发布复核仍检查所有权 |
| 六路源诊断 | status、land、local、attitude、flags、battery 分别记录信封/样本时间、接收年龄、源年龄、拒绝原因和倒退证据 |
| 回调接收记录 | 对比回调时与发布时的源年龄；RMW 接收时间可用时额外记录差值，不可用时为 `null`。只有确认该 RMW 实现使用相同系统时钟 epoch 后，此差值才能辅助判断接收至回调的排队；否则不能据此归因 |
| 分段耗时 | 记录 callbacks、IPC、runtime、snapshot、graph、boundary、reporting，及 snapshot/graph/context 次数；发布对中的三次 context 各自计时 |
| 拒绝与超限日志 | 拒绝时记录六源证据、当前轮本地 target/heartbeat 执行情况；超限提示限制频率，避免每 tick 大量写盘 |

snapshot、graph、boundary 是包含关系，诊断耗时存在重叠，不能直接相加。`last_completed_cycle_work` 是上一完整周期；故障当轮应看 `cycle_work`、`pair_context_calls` 和 `pair_context_seconds`。这些指标用于缩小定位范围，不能单独证明 ARM 调度停顿的根因。

本轮不引入 graph guard-condition 缓存；新增端点或竞争控制源仍由每次 fresh graph 查询拒绝。因而不存在“缓存已验证、忽略后续 graph 变化”的新许可窗口，也不承诺消除全部 graph 查询开销。

## 6. 暂时失鲜与严重锁存的边界

严重故障处理保持不变：已发生部分目标/心跳发布、时钟/样本倒退、解锁或控制权不确定、已进入实际飞行流程，以及既有 publication fault，均不能由暂时失鲜恢复路径清除。

已实现的窄恢复路径为：仅在 `RC_PRESTREAM`/等待手动解锁的地面准备阶段、本次尝试未观察到解锁意图、未捕获导航沿且当前不在导航档位、尚无解锁或 Offboard 命令记录、该轮两个发布 sink 都未执行时，对能够明确归类为 local/attitude 源年龄到期的情况撤回本轮准备。代码对解锁意图保留历史，但不把所有历史导航信号都作为独立锁存量。适配层仍要求新鲜接收及原始样本推进；其余状态、land、failsafe、RC、battery、所有权、地面静止、重置计数和基准一致性仍须有效。不能把源时钟跳变、RC 失鲜或单纯“当前 arm 开关为 OFF”视为足够证据。

节点实际等待原因是 `RC_PREPARATION_SOURCE_AGE_EXPIRED_WAIT_NEW_DATA`（`preparation_age_wait_`），不刷新源戳、不清空倒退记录、不清除严重故障、不继续发送目标；条件恢复后重新经过完整预发送约束。节点直接调用 `only_preparation_source_age_expired()`；其 0.49/0.50/0.51 s 分类、新鲜接收、原始样本推进及其余四源不可失效的条件已有针对性回归。测试夹具自定义的 `RC_PREPARATION_LOCAL_ATTITUDE_DATA_EXPIRED` 不是节点输出字符串。arm ON→OFF 的历史不能被最后一帧 OFF 掩盖。已发生部分发布时依旧锁存，不能走这条路径。

## 7. 验证记录

修复前已在本地完成以下离线检查：

- 11 个相关原生 C++ 测试分别编译运行通过：`test_r62c_review_clock`、`test_authority_boundary`、`test_m1_safety`、`test_socket`、`test_rc_reprepare_admission`、`test_control_diagnostics`、`test_publication_boundary`、`test_publication_runtime`、`test_r6_publication_audit`、`test_socket_boundary`、`test_r62c_review_diagnostics`。使用 C++17、`-Wall -Wextra -Werror -pedantic -pthread`，产物位于临时目录。
- `PYTHONPATH=tests PYTHONDONTWRITEBYTECODE=1 python3 -m unittest test_navigation_service -v`：17 项通过，包含当前反馈失鲜时 WAITING 不被历史 SAFETY 覆盖的约束。

新增 `test_offline_timing.cpp`、`test_rc_preparation_defer.cpp` 和控制诊断测试扩展，重点覆盖：

- 不同稳定日历 epoch 下相同源年龄、真实旧样本和冻结样本不可续命。
- 0.49 s 在最后一次边界变为 0.51 s：已发布目标后必须阻断心跳并保留部分发布故障。
- 主机时钟前跳、后退、单调时间异常、PX4 样本重置及进程重建。
- 本轮回调或 snapshot 使预算超过 0.3 s，包括 0.336 s 情形；发布前和发布后超限分别处理。
- 无解锁意图的地面准备暂时失鲜撤回；解锁意图、RC 失鲜、重置、竞争源、部分发布及严重锁存均不得借此恢复。

修改后已取得以下结果，日志位于 `/tmp/fc01-stability-20261107-tests/`：

| 验证 | 本轮结果 |
| --- | --- |
| 原生 C++ 回归 | 62 个测试执行文件全部通过 |
| 独立复现/回放/适配器 | 3 个独立复现、C11/C12 回放及 adapter API 编译检查通过 |
| native Python | 首次 34/37 通过、3 项因缺失 `native/VERSION` 报错；补齐后 37/37 定向重验通过，零 skip/failure/error、退出码 0，耗时 11.028 s |
| 最终源年龄分类 | 与节点共用 helper 的 `test_offline_timing` 在最终修改后严格重新编译、7 组全部通过，覆盖新鲜接收/原始样本进展/其余四源失效拒绝 |
| 主 Python | 发现 213 项，实际执行 211 项通过；2 个重复调用 native 完整套件的 wrapper 在本次运行中跳过，所包装的测试已单独执行，不以跳过冒充通过 |
| 真实 ROS 2 Humble 构建 | x86 环境、实际 `rclcpp 16.0.19` 与现有 FC01 `px4_msgs`，`jikl_fc01_system` 构建通过 |
| 真实 ROS 进程 smoke | 最终脚本语法检查通过；localhost 合成反馈与接收端运行真实节点：12 个零推力 target、12 个 heartbeat，VehicleCommand 为 0，非零推力为 0，停反馈后进入 SAFETY，退出码 0；退出 drain 未见新增输出 |

在工程根目录复现离线回归的常规命令：

```bash
bash native/scripts/test.sh
python3 -m unittest discover -s native/tests -p 'test_*.py' -v
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

第一条已包含 native Python，第二条用于单独重验该部分。本轮主 Python discovery 仅在运行器内跳过了已被第一条覆盖的两个 wrapper，未改测试文件；以上常规第三条会照常再执行它们。整套可重复验证，无需在香橙派现场重复这些开发机回归。

真实 ROS 构建复现命令：

```bash
source /opt/ros/humble/setup.bash
source /home/jikl62/fc01_real_ws/install/setup.bash
colcon --log-base .build/offline-stability-20261107/log build \
  --base-paths native/src/jikl_fc01_system \
  --build-base .build/offline-stability-20261107/build \
  --install-base .build/offline-stability-20261107/install \
  --packages-select jikl_fc01_system --event-handlers console_cohesion+ \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

首次失败对应的 `native/VERSION` 在 Git 历史中从未被跟踪，20261103 源码归档也缺失，但安装、导出、部署工具和测试直接读取它。`package.xml` 从已核对基线至今均为 `0.4.6`；已在独立提交 `3e259cc` 以该值补齐元数据，不改变部署逻辑，也不声称还原了内容未知的旧文件。重验日志为 `native-python-fixed.log`；helper 最终重编译日志为 `offline-timing-classification.log`。首次完整脚本退出码 1 的记录保留，补齐后仅定向重跑受影响部分，未以再次全量执行重复已通过的 C++ 测试。

真实 ROS smoke 的 JSON 为 `.build/offline-stability-20261107/ros_offline_timing_smoke.json`。测试固定 `ROS_DOMAIN_ID=187`、localhost 和全部飞行动作门关闭，运行 `ATTITUDE_READY` 诊断预发送，使用本机合成反馈，不启动 Agent 或连接 PX4。代表性观测如下：

在上文已 source ROS 与消息工作区且构建完成的开发机环境中，显式运行：

```bash
/usr/bin/python3 tests/ros_offline_timing_smoke.py --report .build/offline-stability-20261107/ros_offline_timing_smoke.json
```

| 本机合成输入阶段 | 实测结果 |
| --- | --- |
| fresh local | 信封/样本/接收年龄分别为 14.747/16.747/0.098 ms |
| PRESTREAM | 三次 context 为约 21.2/17.4/17.3 μs；4 次 snapshot、84 次 graph 查询，graph 合计约 81.1 μs；周期样本约 50.094 ms |
| 停止反馈 | 约 0.602 s 后轮询观测到 SAFETY；local 信封/样本/接收年龄为 550.589/552.589/501.156 ms，源时钟/local/attitude 新鲜度均失效，`publication_fault_latched=false` |

0.602 s 包含采样和状态轮询延迟，不是新的超时阈值。这里的低 graph 耗时属于本机 x86、合成拓扑的一次观测，不能解释或排除 ARM 现场的 336 ms 停顿；这些本机时间年龄也不能解释现场约 0.489 s 偏移。窄 RC 准备恢复的约束由对应离线回归验证，本 smoke 不宣称覆盖真实 RC 起飞。

离线通过只证明对应测试路径和构建成立；ARM 周期表现、现场源年龄偏移、真实 PX4 采用及飞行均未验证。

## 8. 一次有界拆桨地面验收

此步骤供用户后续手动安排，不表示本轮已经授权部署或启动飞行任务。

1. 拆桨、保持未解锁，确认实际运行配置 `arm_enabled=false`、独立遥控解锁策略和唯一控制进程。保留现有 DDS V3、原 DDS Agent 与第二 UART 配置；不用重复 CRC 接收或香橙派回环测试。
2. 通过现有人工服务流程选择待验收二进制，留存运行文件哈希、实际配置、进程 PID 和启动时间。若这些证据不匹配开发提交，先停止本次软件验收；服务切换不由本轮自动执行。
3. 在无 Wi-Fi/NTP 的既有离线条件下观察一次 30–60 s。保持安全遥控状态，不发起解锁、起飞、导航或追击任务。保存状态与 stderr，检查六路 `source_timing`、主机时钟增量事件、`last_completed_cycle_work`、graph 查询次数和端点所有权结果。若自然出现准备拒绝，保存当次 `publication_blocked`、`cycle_work`、`pair_context_calls` 和 `pair_context_seconds`；未触发发布路径应如实标为该路径未实机验收。
4. RC 失鲜/接管只由现场人员在拆桨安全状态下按原有操作输入核验，不能由自动脚本制造解锁或飞行状态。部分发布、真实时钟跳变及 336 ms 停顿的强制注入仅在离线 mock/回放测试中进行。唯一控制源变化的拒绝由离线回归验证；现场真实发现竞争源时，应拒绝控制并保留证据，不为测试自动启动第二控制节点。
5. 区分结论：构建通过、进程运行、反馈有效、发布边界通过、PX4 采用、真实起飞是不同验收项。此次观察最多证明地面软件路径，不宣称自主飞行能力。

## 9. 根因仍未确定时的最小取证

只采上述一次观察中的软件诊断，不重新开展全套串口测试：

| 待区分原因 | 最小证据与解释范围 |
| --- | --- |
| 源样本到达前已老 vs 回调后被拖慢 | 同一路 `callback_envelope_age_s`/`callback_sample_age_s` 与拒绝时源年龄、接收年龄；到回调时已老不能单独证明网络延迟，也可能是时间映射偏移 |
| 回调排队 | `rmw_to_callback_s`、callbacks 总耗时；仅在已确认 RMW 接收时间与节点系统时钟使用相同 epoch 后，前者才有排队解释力，否则保持未知 |
| 疑似主机时钟变化 | `host_clock_*` 事件、系统与单调增量差；这些值混合了时钟变化和采样调度延迟，不能仅凭事件或错误日历日期定因 |
| snapshot/graph/IPC/报告耗时 | 同一故障轮 `cycle_work`、`pair_context_calls`、`pair_context_seconds` 和 `last_completed_cycle_work`；注意包含关系和调度空白 |
| 同步偏移 vs 真实传输/生产延迟 | 若所有源在回调入口已呈稳定共同偏移，仍需与当时 PX4→主机时间映射证据交叉核对；现有 connected/timesync 日志仅是背景，不足以单独排除偏移 |

若没有出现原故障，只能报告本次有界观察未复现，不能宣称 .489 s 或 336 ms 的历史根因已消除。

## 10. 合并与回退范围

修复前审计为 `e2b90d0`，缺失版本元数据修复为 `3e259cc`，控制运行时、诊断及直接相关回归为 `abaea50`（`fix: enforce complete offline control timing budgets`，11 个文件）。建议按 `e2b90d0` → `3e259cc` → `abaea50` → 最终报告提交的顺序合并到主线，方便独立审查和回退。

纯结构调整为持久 executor 和 graph 查询代码归集；行为修复为完整本轮预算、边界末端复核与窄地面准备恢复；诊断不授予控制权限。保持 UART 拓扑审计提交历史，不重写或回退已有串口配置，不纳入用户 `fc01_config_check.cpp` 的格式编辑。

本轮文件范围如下；其中头文件路径前缀为 `native/src/jikl_fc01_system/include/jikl_fc01_system/`：

| 文件 | 修改用途 |
| --- | --- |
| `attitude_altitude_core.hpp` | 严格限定地面准备撤回条件，保留严重故障 |
| `control_diagnostics.hpp` | 覆盖回调至最终发布的本轮预算 |
| `rc_navigation.hpp` | 保留本轮解锁意图历史，禁止最后一帧 OFF 掩盖 |
| `runtime.hpp` | 运行时的无解锁准备撤回入口，保留命令与故障证据 |
| `source_diagnostics.hpp` | 六源原因/年龄、窄恢复分类和主机时间诊断 |
| `native/src/jikl_fc01_system/src/fc01_system_node.cpp` | 持久 executor、末端计时、fresh graph 查询及诊断接线 |
| `native/tests/adapter_compile_check.py` | 更新实际节点新增 ROS MessageInfo/executor API 的适配编译夹具 |
| `native/tests/test_control_diagnostics.cpp` | 本轮回调、snapshot 和发布预算回归 |
| `native/tests/test_offline_timing.cpp` | 离线时域、源年龄分类、跳变和最终边界回归 |
| `native/tests/test_rc_preparation_defer.cpp` | 未解锁准备恢复和严重故障不可恢复约束 |
| `tests/ros_offline_timing_smoke.py` | 显式运行的 localhost ROS 进程级模拟验证；固定关闭飞行动作门、不启动 Agent、不打开串口 |
| `native/VERSION` | 与已存在 `package.xml` 一致的 `0.4.6` 元数据；独立提交 |
| `docs/FC01_OFFLINE_STABILITY_20261107.md` | 本审计、证据、验证及合并说明 |

合并到主线时逐项核对：`source_clock.hpp` 的阈值和判断保持、`publication_boundary.hpp` 三次复核保持、PID/推力/保护阈值保持、DDS V3 与 UART 配置保持、严重故障与 RC 接管测试通过。需要回退时仅回退本轮软件修复提交，不通过改参数或清故障伪造恢复；香橙派服务版本回退由用户另行安排。本轮没有运行现场服务切换、参数修改或固件刷写。
