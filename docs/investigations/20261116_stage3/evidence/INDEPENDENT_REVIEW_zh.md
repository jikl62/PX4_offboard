# 阶段 3 可见产物独立复核（2026-11-16）

复核对象为当前 `tests/ros_stage3_lifecycle.py`、运行夹具、阶段 2 构建证据及 `/home/jikl62/.cache/fc01-stage3-20261116/accept2-*` 的 11 次完整运行。未复跑 ROS、未启动控制器、未访问串口或真实 PX4。独立 JSON 检查结果位于 [INDEPENDENT_REVIEW.json](INDEPENDENT_REVIEW.json)。

## 结论

可见的 11 次 accept2 运行全部 PASS，原生进程均正常退出 0，全部验收 gate 为 true。没有发现阻止将这些结果作为 **x86 ROS 2 Humble + localhost DDS + 合成 PX4 peer 软件集成证据** 归档的问题。该结论不构成 ARM64、PX4 SITL、真实硬件飞行或物理稳定性放行。

## 独立核验

- 当前 harness SHA256 为 `8e6a54ba16d5796b71071f98a9bc91bded8abef18a41f3f7321fa18033ae6096`，11 份结果均匹配。夹具 SHA256 为 `800202615c8d45e4627c79d308ea4097f15c545ebb45181eaf8451419818f089`，与阶段 1 捕获配置语义一致，键和值无差异。
- 阶段 2 `ROS_BUILD.json` 的全部 47 个生产输入 SHA 与当前树一致，实际测试 ELF 与记录的 `c14dbd9548217df28ceaf46ddc5de57b94a30e8b49dd9af9582668353defc0c0` 一致。11 份结果中的构建证据摘要也一致。产物为 x86-64。
- 逐份读取 `received.jsonl`：消息数量与 result.json 一致；target 与 heartbeat 的完整 timestamp 集合相等；所有目标姿态/推力有限，垂向推力在配置范围内，横向推力为零。
- 10 种 replay 场景全部通过；另有 1 次 normal plant 通过。正常流程及地面 range 不可用流程都完成 READY、人工解锁意图、新 armed/Offboard 状态、Ramp、Ascend、HeightHold、LAND、Disarm、新安全地面和第二次 READY，第二次 READY 继续等待新的人工授权。
- 正常着陆将 ACK 与 DISARMED 状态分离：ACK 后仍维持 armed，验证 ACK 不会替代新 DISARMED 反馈；确认上锁后才收到既有 176/STAB 释放命令。
- 故障场景在健康输入恢复后保留 SAFETY 与首次故障。已完成的 7 个故障场景，在 `latch_preserved_no_more_wire` 之后直到包括清理阶段，原始接收记录中均无 target/heartbeat。SAFETY drain 同时拒绝观察 SAFETY 后才生成的新 timestamp。
- RC 手动接管先验证在 Offboard 中 DESCENT 继续约 0.6 秒并维持实际 DDS 发布，再模拟 PX4 退出 Offboard 验证锁存。它证明受控交接与模式退出后的停发，不证明持续 Offboard 条件下完成整段下降。
- 速度单独偏差场景维持 HeightHold、健康垂向反馈和实际 DDS 发布约 2 秒。独立统计注入到 warning 验证结束的完整边界，heartbeat 最大间隔约 51.11 ms，target 最大间隔约 51.07 ms，均小于 300 ms。
- normal plant 的超调约 0.139 m、稳定窗进入约 7.93 s、HeightHold 进入约 10.12 s、观察到的 hold RMS 误差约 0.127 m；这些数值只属于既定线性单轴模型。replay 的零误差由预设输入产生。

## 归档时必须保留的解释

- 本机测试只使用固定 domain 191、localhost 和合成 peer；没有 PX4 固件、Agent、UART、电机或物理测距参与。测距字段由合成数据生成，不能证明 MTF 测距精度、真实 EKF 独立性或实际地面可用性。
- XY 固定为零，水平漂移零不表示实机抗漂移能力。plant 没有姿态、噪声、延迟、地效、电机或外部扰动模型；hover thrust 恰好等于模型参数。
- 故障延迟是主机注入到 IPC 观察 SAFETY 的时间，包含反馈周期与状态采样，不是 PX4 实际检测、失联处理或退出延迟。
- reset 场景恢复 reset counter 为零是对锁存的对抗性测试输入；真实估计器 counter 不应以此方式回退。
- 清理期间跳过“当前合成 armed=false 则不能接收任何非零旧包”的校验，用于排空合成状态切换前的在途 DDS 包；有限值、推力范围和姿态合同仍校验。清理不属于飞行性能窗口。
- 原目录中早期失败尝试须保留。特别是 `verified-velocity_warning-replay` 为清理竞态导致 FAIL/native_exit=-9，`final-feedback-stale` 为 native_exit=-9；不能将这些重命名为成功或混入 accept2 结果。

## 可选改进

当前速度告警场景在持续观察期间验证消息数量，但通用连续性 gate 只统计到故障/告警注入时刻。本轮已独立核查告警后窗口最大间隔约 51 ms，因此不影响这批原始证据；后续可将注入后的持续告警窗口也纳入脚本的最大间隔、单调戳和配对 gate，避免仅凭计数漏过短暂停发。
