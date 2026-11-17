# Stage 3 最终 ROS 生命周期夹具独立审查

审查时间：2026-11-16。只读审查 `tests/ros_stage3_lifecycle.py` 和 `tests/fixtures/stage3_runtime_20261114.yaml`，未启动 ROS 节点、域、Agent、串口、服务或硬件；未改生产源码。额外执行 `python3 -m py_compile tests/ros_stage3_lifecycle.py`，退出 0。

审查版本：harness SHA256 `6797edb854c873fe68a3e69112be6689938043c84e3c92e0aa873d9f64de147e`，fixture SHA256 `800202615c8d45e4627c79d308ea4097f15c545ebb45181eaf8451419818f089`。Stage 2 ROS_BUILD.json 中 47 个生产输入与当前文件 SHA256 全匹配。二进制哈希由夹具执行前核对；本子任务未独立运行该 ELF。

## 需要修改或缩小声称范围的发现

### P2：reset 故障的所谓健康尾帧仍反复携带故障条件

位置：harness 338-339、376-384。

quat_reset / vertical_reset 注入将相应 counter 改为 1；尾帧 `simulated.update(...)` 并未恢复 counter。生产参考仍保留任务起点 counter 0，因此 counter 1 的后续帧仍触发 ACTIVE_*_RESET。即使回归使锁存失效、每次只临时拒绝当前 reset，夹具也会 PASS，不能证明“健康帧不能清除首因”。

建议：锁存专用尾帧显式恢复注入前 counter（本机故障注入，目的就是去除当前故障条件），继续维持 armed+Offboard，并验证原 first reason 与无新输出。也可保留 counter=1 的连续故障测试，但其结论只能是“重复不匹配参考仍拒绝”。

### P2：0.2 秒无条件排空窗口可能掩盖故障后新发布

位置：harness 374-384。

发现 SAFETY 后恢复反馈，前 0.2 秒出现多少 target/heartbeat 都不判失败。DDS 确可能有旧消息排队，但当前算法未区分旧排队消息和 SAFETY 之后重新生成的消息。因此一次短暂恢复旧流/重发随后再静默也能 PASS。

建议：记录 SAFETY 观察时的同步源时钟与实际消息 timestamp；允许有界排空已经在途的旧戳消息，任何晚于故障锁存边界的新 target/heartbeat 都失败。至少保存并审查 0.2 秒期间收到的所有消息，不能只检查随后的计数相等。

### P2：心跳连续性指标遗漏窗口两端，且未覆盖 target 连续性/配对

位置：harness 419-443。

只计算已收到 heartbeat 的相邻内部间隔，没有 `nav_at -> first heartbeat` 和 `last heartbeat -> heartbeat_end`。末尾在注入前已停流一段时间，或开始延迟，却无后续 heartbeat 形成大间隔时，仍可满足预算。target 只有值域检查，没有相同窗口内的最大间隔、首尾覆盖或与 heartbeat 的配对检查；plant 永久使用最后收到的 thrust，间歇缺失 targets 可以被模型缓存隐藏。

建议：两类消息均计算完整窗口首尾和内部间隔；要求严格递增 wire timestamp、固定合理目标/心跳数量并按 source timestamp 一一匹配。故障静默窗另算，勿把有意停止纳入正常调度失败。报告的当前 max gap 应明确只是已收心跳的内部间隔。

### P2 覆盖限制：LAND/Disarm ACK 和新状态没有被分别验证

位置：harness 119-131、318-326；生产 Core RequestDisarm 分支约 1901。

每次收到 Disarm 就立即同时发送 accepted ACK 并把模拟 armed 改 false；主流程等待的是本地模拟字段，而不是实际节点确认新 DISARMED 状态。随后又在该 status 观察前设置离开 Offboard 并拨安全开关，真实节点可能直接走 takeover + 安全重准备，绕过待验证的正常 RequestDisarm 终态。因此这能证明“发出接地 Disarm 后重新准备”，不能证明 ACK 不能代替独立新状态、正常终态完成、无 ACK 时的新状态路径或 ACK 归属处理。

建议：至少将 ACK 与 DISARMED status 分开：ACK-only 后维持 armed 一段时间，要求未终结/不重发；之后提供独立新 status+land，确认节点终态或已确认 safe-disarm 证据，再人工释放模式。其他 ACK 归属/超时组合可以明确列为现有单测范围，不冒充本轮 DDS 覆盖。

### P3：RC 请求接管只检查瞬时 DESCENT，未验证其持续发布合同

位置：harness 357-360。

看到 DESCENT 后立刻提供 PX4 离开 Offboard，未持续观察仍 armed+Offboard 时的受控下降输出。因此此场景能证明阶段经过 DESCENT，不能证明在等待真实接管期间持续新鲜受控目标/心跳。建议维持至少 0.5 秒，检查 release_requested、原下降 reason、持续 wire 和无 Arm/176/400，然后再模拟真实离开 Offboard。

### P3：报告哈希在结束时读取，无法独自证明运行期间文件不变

位置：harness 455-457。生产文件只在启动前校验。父任务如在场景运行期间修改 harness，会使结束时的哈希指向未实际执行的新字节。建议在启动时保存 harness/fixture/生产摘要并在结束时重核；不一致就 FAIL。本轮已完成结果只能绑定真正执行的版本，修复夹具后应重跑受影响场景。

## 已确认合理之处

- 非零输出确由实际 DDS 订阅回调读取；控制输入为真实 ROS 2 messages，原 IPC STATUS 读取真实节点状态。未用本地 publish 返回冒充接收。
- 固定 localhost/domain191、删除外部 DDS 配置、启动前检查其他节点。脚本没有串口/Agent/systemctl/PX4 参数或真实硬件路径。节点启动参数为本机私有临时 socket 和独立 runtime；不会覆盖运行发行目录。
- fixture 保留人工 RC 入口、arm_enabled=false、目标1.5m、visual=false；hover/reserve/min/max/slew=.50/.04/.12/.80/.40，实际部署控制参数未放宽。旧构建 ELF 和生产输入 SHA 被双重核对。
- 阶段顺序门不只是成员存在，还检查第一次出现的顺序；未人工 arm 时非零目标会失败；任何非 Disarm command、空中或高速 Disarm 会失败。
- 正常模型指标明确标为简化单轴模型，回放与输出无因果；XY固定值明确不能证明真实水平漂移。指标排除清理期间瞬移地面和故障后故意停流，这一点正确。
- 速度差场景在警告后观测 2 秒，要求每条 STATUS 仍 HEIGHT_HOLD/feedback healthy、至少30条 target 和 heartbeat，能够发现仅速度差导致持续停机。
- 故障原因与生产合同相符；RC杆量干预先请求 DESCENT，再真实模式离开锁存，而非把请求接管直接写成立即静默。

本次审查没有发现生产代码变更或物理 IO。上述问题属于测试结论的覆盖/强度问题，不能据此说生产保护已经失效，也不能把本机模型数据写为 PX4 SITL、ARM64、真实动力或实际飞行性能结果。
