# C：R6.1 最小恢复保管修复独立复审任务

仅离线审查、解压、编译、注入；不得连接或操作真实 FC01，不得修改 B/C 原始断言，不得发送真实控制或参数。用户报告 Disarmed `15→14→15`，Z/VZ valid、XY/VXY invalid、land~1Hz；本轮不重做基础 DDS/176/ACK/准备/地面释放开发。

## 输入与重放

交付 `R6_BASELINE_FULL_SOURCE.zip` 为精确 R6 归档；`R6_1_FULL_SOURCE.zip` 为本次完整源码；`R6_TO_R6_1.patch` 为增量。基线 R6 的 source manifest 保留在基线归档中；R6.1 使用自己的 `R6_1_SOURCE_SHA256.txt`。先核对外层 SHA256 与两份源码 manifest，再在独立 R6 解压副本 `patch --batch -p1 < R6_TO_R6_1.patch`，逐路径、逐字节与 R6.1 比较。本轮没有 git commit SHA，以 manifest 为版本身份。

## 必须复测的肯定式性质

1. 原 C-S10 positive `timeout`、`shutdown`；原 `C_R6_EXTENDED_HEALTHY_GATE.cpp` 全部字节保留。R6 必须重现约 38.05s RED，R6.1 必须 GREEN，不能调整配置中的恢复预算规避。
2. `tests/test_r6_1_recovery_custody.cpp`：正常/正常关闭/console 断连，各 600 秒虚拟时间且 1Hz detector；仍 Armed+Offboard 时两个最终 sink 每周期一次、原限幅/slew、absolute deadline 不续期、无 blind Disarm/21/176。明确这不是飞行动态模拟或实际 FC01 控制力证明。
3. 预算之后 invalid/stale Z/VZ、quat/reset、Z/VZ reset、graph conflict、Kill、failsafe、clock regression、battery warning、stall 的原拒绝条件；四项 final guard 否决。不得通过复用输出或预算新逻辑越过它们。
4. 21 ACK 接受/拒绝/缺失、提交与模式证据分开，预算后仍未退出 Offboard 时不主动停流、不重试。无 XY/VXY 时保持原 21 拒绝合同，不凭 ACK/nav_state/evidence ID 声称 task 已接管。
5. 晚到 1Hz ground proof、单 landed pulse/bounce/frozen/replayed/gapped stamps；原 ramp-to-zero 与 at_rest、两个 NEW stamps、state-only Disarm 和 ground release 不弱化。R4 Command400 跨事务旧 ACK 隔离、history、final commit 与禁止盲重试必须通过。
6. 重新运行 `scripts/test.sh` 并捕获完整退出码；保留跳过原因及历史漏洞探针语义。实际 Node API fixture 仅属窄语法证据；不能签成 native Humble build。

## 源审查重点

生产代码仅 Core 的截止处置/诊断与 Node 两个逾期 JSON 字段发生变化。核对 Runtime/ACK、Authority、vertical_controller、final guard/boundary、235 px4_msgs 字节相同，全部 YAML 参数解析值相同。四项默认 gates=false、动力参数 unset、Land fallback=false 不变。

审核截止的语义变化：它是恢复完成 deadline，耗尽后锁存并保留健康保管，不是有限时机动保证。所有原条件每周期重检、没有重置 slew/integrator 或创造额外权限。没有实际后备时，不能把另一计时器耗尽当作可安全退出的证据。请把“此代码停流缺陷已关闭”与“整体恢复/物理飞行安全通过”分开签署。

正常 Node 关闭 60s 后健康 Armed+Offboard 继续输出的旧规则沿用；真正进程停止/异常/失流没有 Companion 保证。请独立检查 Node final guard 重采样、部分 publisher 失败后的记录，注意原有同 stamp 改写在纯 Core fixture 可注入、真实 ingress 的 Stamp.accept 合同不变。本版不扩大或重写该接口。

## 仍需外部证据的边界

Ubuntu22.04/Humble/pinned DDS 原生构建；真实 PX4 receipt/ULog 与 Companion commit 比对；当前厂商固件实际 fallback flight task、无 XY 前提、动力/电池、RC/Kill、SIGKILL/供电/Agent 断开。均不得用本轮离线通过补齐。

输出分项 C 结论：源码、补丁重放、原生构建、用户已有 Disarmed 接口证据、动力/完整自主飞行。没有相关外部证据时保持对应 UNKNOWN/NOT VERIFIED/NO-GO，不要求 B 重复开发已经验证的基础路径。
