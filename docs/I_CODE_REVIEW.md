# 独立源码复核及修正

复核对象：supplied 0.4.1 baseline commit `770e22beb4113e25ffbce3fb00f397517d86b88d` 到冻结实现 `966a18cf5bd96346970c63e293af49ae0b4bc77d`。一名 fresh reviewer 只读审查全部修改、契约、真实发布边界和测试，并在源码树外运行复现。初始结论 **BLOCK — OFFLINE SOURCE ONLY**；无 Critical，三项 Important、一项 Minor。源码、index、HEAD 未被 reviewer 修改。

| 级别 | 实际发现 | 实现方复现与修正 |
| --- | --- | --- |
| Important | 首对 InputRc/switch 消息就令两个 readiness=true；在独立打开的合成飞行门/RC confirmation 下，最终假 Arm sink 可达 | `test_authority_review.cpp first` 先 RED；要求原始信号/sample 至少两次推进，且两次接收均在原 freshness 窗口内；首帧、仅 envelope 推进、stale gap 后首帧拒绝。两次推进的合成正向路径仍可达假 Arm sink |
| Important | takeover 后再发生普通故障，原 reset 仅检查 takeover latch，因而可清故障 | `fault` 先 RED；新增独立且只增不减的 restart_required 状态。普通故障无论先后顺序都只能重启，STOP/LAND/TAKEOVER/reset 不能消除 |
| Important | 未读取 switch_changes，丢失中间 ON 数据后，收到位置相同但计数改变的 OFF 数据仍继续发布 | `counter` 先 RED；采样位置或 source counter 改变均增加 generation。最终双 sink 在 Core step 前拒绝旧输出，随后锁存并不重入；counter 回绕/重置亦保守释放 |
| Minor | 已收到 Kill 等事实但尚未 step，authority_ready 仍为 true；TAKEOFF admission 也可先接受，最终 Arm 正确拒绝 | `diagnostic` 先 RED；Core 的同一 takeover-cause 判断用于 step、takeoff admission、JSON。Kill、显式事件、failsafe、已建立的 Offboard loss、活动开关变化立即报告阻断；普通故障优先报告 restart required |

修正后的四组 targeted 回归及完整套件通过；证据见 `I_REVIEW_REGRESSIONS_RED.txt`、`I_REVIEW_REGRESSIONS_GREEN.txt`、`I_TEST_OUTPUT.txt`。真实 JSON 另由 Python 编译 Runtime emitter 后解析并验证。`I_REVIEW_ORIGINAL_REPRO_OUTPUT.txt` 保存初始复现输出，不是最终结果。

未请求第二轮独立复核；以上是单次审查后的实现方回归与不变量验证，不宣称 reviewer 已审查最终 archive/replay。

Reviewer 拒绝判断：真实 Humble/pinned-message 编译、DDS/topic、RC 校准/物理 takeover/Kill、PX4 failsafe 效果、Orange Pi、飞行动力学/数值调参、被跳过的原生 socket。上述均 UNKNOWN。同 UID INTERACTIVE 按提供的基线信任模型评估，不宣称强认证。没有真实硬件动作或飞行授权。
