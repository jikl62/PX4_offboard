# 第二阶段确认问题及未应用候选

只有具有可复现证据的条目登记为问题。分级针对当前影响范围；默认视觉关闭不能被描述成已经发生空中控制失效。

| 级别 | 条目 | 证据与处理 |
|---|---|---|
| P1，已修复 | 视觉适配器 SIGINT/SIGTERM 与订阅回调交错，异步关闭 context 或中断取消息可导致 RuntimeError/RCLError | 5198d33 SIGINT 实测复现；旧候选 SIGTERM 失败；直接抛 KeyboardInterrupt 候选 1/6 轮仍失败。`df735d6` 独立提交使用主线程停止标志，回调结束后清理。14 次真实 ROS 冒烟（7 SIGINT、7 SIGTERM）通过，两组各 10 次同进程重启、活动异常传播及处理器恢复通过；安装后另测 SIGINT。 |
| P2，未应用候选 | `navigation_probe.probe` 最终写证据抛 OSError 时，后续 destroy/shutdown 被跳过 | baseline/current 受控假 ROS 均复现：写盘错误保留，但 context 活跃、无资源清理事件。`candidates/probe_cleanup.patch` 单独提供；5 场景验证原写入错误继续向上传播，候选执行两项清理。未改生产，以免把行为差异混进重构。 |
| P2，待独立修复 | 适配器节点构造中途因未配置身份失败，没有显式销毁已经构造的基类 Node | baseline/current 真实 ROS 均创建 1 次、显式 destroy 0 次；context 已关闭，但探针保留引用的 `get_name()` 仍可调用。只证明缺显式销毁，未证明永久内存泄漏。需要明确构造失败资源所有权，作为单独候选处理。 |
| P2，待视觉合同 | 视觉状态及指令的来源字符串没有长度预算 | baseline/current 纯核心探针均接受两个各 1 MiB 的 source_id，current/active 成立。证明缺预算，不证明默认视觉关闭会崩溃，也不证明 Orange Pi 的实际耗时。需要协调消息/适配器/inbox/controller，不能本轮暗加新拒绝门。 |
| P3，验证器假设，已纠正本轮校验 | 缓存安装验证把 CDR 再序列化全字节相同当作字段往返相同 | 首次旧脚本失败；500 次探针均字段相同，仅一次 offsets 8..11 不同，属于 uint16 schema 后 uint64 sequence 的对齐填充。新校验按完整 schema 定位所有 padding，505 条消息两轮共 1010 次字段/类型/浮点位模式精确通过；改填充仍相等、改有效字段必须检出。生产消息和序列化实现未改。 |

没有新增已确认 P0。结构较长、同步 CSV、未实测 ARM64 时延等属于维护/验证缺口，未凭静态猜测登记为已发生的逻辑故障。

原始失败和通过记录见证据 ZIP 分卷与 `evidence_index.json` 内 `vision_shutdown/`、`python_services/`、`cpp_control/vision_identity_budget_probe.*`、`ros/`。部分视觉动力链测试开发时的中间日志被同名最终成功日志覆盖，`development_attempts.json` 只保留了失败条件和诊断摘要；不能说每个中间失败的原始完整日志均仍在。

本轮测试夹具还纠正了三类假设：合成人工 RC 参数必须满足原配置合同；临时 socket 父目录必须显式 0700，不能依赖其他测试改过的 umask；不同进程的诊断 PID 应作为受控输入，而真实进程所有权不得 mock。根差分基线/当前的真实字段初次比较仅 PID 不同，修正夹具后逐字节相等。这些不是生产修复。

TSan 已实际编译并尝试运行，退出码 66，`FATAL: ThreadSanitizer: unexpected memory mapping`。它没有完成竞态检测，不能与 ASan/UBSan 的通过结果混算。
