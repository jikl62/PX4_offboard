# FC01 R6.1 最小恢复保管修复

版本：`0.4.4-offboard-flight-dev-r6.1-custody-fix-dev`。基线是完整 R6，独立目录，未覆盖 R6 或用户安装版本。

本版解决：恢复预算耗尽时，在反馈、Authority 和发布门控仍合格且 PX4 仍 Armed+Offboard 的情况下，Companion 主动进入 Safety 并停止 target/heartbeat 的确定性代码缺陷。原 C 扩展断言保持原字节，R6 在 38.05s RED，R6.1 GREEN。

恢复截止时刻不延长、不重置。耗尽后锁存 `recovery_budget_exhausted=true`，继续使用原反馈下降控制器、限幅、slew 和最终双 sink，明确记录交权未验证。ACK 接受/拒绝/缺失、控制台断连及正常关闭请求都不能单独证明交权。

**DEV / CONDITIONAL，真实有桨自主飞行尚未放行。** 这不是保证有限时间落地的策略，也不是无限维持推力安全的主张。健康状态持续而没有落地或模式退出时，保管可以持续超过截止时刻；当前缺乏经验证独立后备时，另一个计时器同样不能证明可以安全停流。估计失效、reset、Kill、Authority 丢失、饱和、stall、publisher 异常及硬进程终止仍需独立后备，原拒绝条件保留。

用户本轮报告真实 FC01 全程 Disarmed 的 `15 → 14 → 15`，Z/VZ 有效、XY/VXY 无效、Land Detector 约 1Hz。本版沿用这一接口约束；不重复 DDS、Command176、基础 ACK、准备和地面释放开发。不将用户报告推算为 Armed、动力、落地或失流后备证明。本轮未连接或操作真实飞机。

阅读：

- `docs/R6_1_CHANGELOG.md`：精确修改范围和历史测试期望更新。
- `docs/R6_1_SAFETY_EVIDENCE.md`：离线证据及不能证明的边界。
- `docs/C_R6_1_REVIEW_TASK.md`：C 独立复审入口。
- `R6_1_SOURCE_SHA256.txt`：本版本当前源码校验表；历史 R6 校验表仅作基线记录。

仅离线验证：

```bash
bash scripts/test.sh
bash scripts/r6_no_props_check.sh
```

不含真机部署、解锁、模式、参数或飞行操作步骤。真实默认四项动作 gates 仍为 false；推力/gain/slew 参数仍 unset。1Hz detector 的现有最大 gap 与新样本证明不变。原生 Ubuntu22.04/Humble 构建与真实 Armed 场景未执行；实际 Node 仅完成窄 type/API fixture 的语法编译。

交付补丁 `R6_TO_R6_1.patch` 对完整 R6 使用 `patch --batch -p1` 重放，结果与本版完整源码逐路径、逐字节比对。交付包同时包含准确 R6 基线压缩包，便于 C 重放。

历史 `README_R6_FIRST.md`、R4/R5/R6 文档与原 C 反例保留；它们不是 R6.1 的当前测试结论。
