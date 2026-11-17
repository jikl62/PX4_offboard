# 阶段 2 独立复核

基线 `3677b9dd1f35e092e43501d26164c6263545c947`。本审查只读生产文件；独立复现源码与二进制写在 `/tmp/`。没有设备访问、部署、远程操作或飞行程序启动。

## 确认问题

**P2：启动期临时拒绝会污染真正任务故障的首因诊断。**

位置：`native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp:1140`（`if (!queued_safety_evidence_.recorded)`）。

`observe_safety_event` 尚未形成解锁意图时，也会把预检未通过等通用条件写入 `queued_safety_evidence_`。随后 `rc_startup` 调用 `RcNavigationStart::step`，其原规则允许未解锁准备失败 reset/重新等待；不是每个 queued reject 都锁存 Safety。该成员却不在健康准备成功、READY 或人工解锁时清除。于是正常启动期短暂 `preflight=false` 保存 mask16；健康恢复、READY、人工解锁后真正 `failsafe=true` 锁存 Safety 时，日志仍给出 mask16 及旧源水位。这个缺陷会使此次新增诊断误导事故首因判断，虽然不改变控制许可。

最小复现：`/tmp/fc01-stage2-review-queued-repro.cpp`，使用实际 Runtime/Core 和现有 VisionFlight 合成输入。命令：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include -I native/tests \
  /tmp/fc01-stage2-review-queued-repro.cpp -o /tmp/fc01-stage2-review-queued-repro
/tmp/fc01-stage2-review-queued-repro
```

本次实际退出1：

```text
early phase=0 mask=16
fault phase=19 mask=16 stamp=1000 expected_failsafe_bit=2 current_status=3851000
```

建议只保留确实引起本任务 Blocked/Safety 的首因，或明确在新的安全准备生命周期重置历史临时诊断；增加“启动短暂拒绝→健康准备→人工解锁→不同排队故障”的回归。不要改动原准入/锁存判断来修日志。

## 非阻断说明

- `RangeTrendDiagnostics` 使用固定标量状态，没有队列扩容、动态I/O或资源所有权变化；纯观测入口未向Core传入范围测量，不授予动力。
- `TakeoffEvidence` 每次观察追加 motion_age 检查，补上无新local时的期限缺口；原阈值、源域检查、有效期限内配对与RAMP托管保留，未见进一步阻断问题。
- `queued_safety_mask` 与原布尔或等价；各谓词只读，全部求值不引入已发现副作用。Kill同时设置Kill与rc_live失败位是如实记录多个失败条件。
- `range_sample_stamp` 是最新local样本，而差分字段在两个200ms窗口之间保留上一完整区间。建议报告明确这是最近已完成的区间，不把它解释为当前每条local的新差分。若后续做精确跨日志对齐，宜另存comparison起止源戳；本轮无需为此改控制。
- 该范围值是PX4地形/底面距离估计，不是已认证MTF原始数据，也不保证独立于融合z；头文件已准确声明。缺范围或仅FLOW位不得当作range比较。
- 未发现姿态重置/心跳保护需放开的证据，保持现有异常拒绝。

## 独立实际执行

1. `native/tests/test_20261116_diagnostics.cpp`：C++17/O2/Wall/Wextra/Werror/pedantic编译与运行0，输出 `DIAGNOSTICS_PASS 10 queued cause bits + range trend/reset/freshness/invalid`。它当前没有覆盖上述启动期污染路径。
2. `native/tests/test_20261116_takeoff_expiry.cpp`：同标志编译与运行0，输出 `TAKEOFF_EXPIRY_PASS helper boundaries + Runtime late detector`。
3. `git diff --check`：退出0。
4. `python3 -m unittest test_receive_event_adapter -v`：1个Python unittest，运行退出0，包含真实SystemNode适配器多组C++断言；输出 `Ran 1 test in 2.598s; OK`。

这些是离线断言，不是实际DDS/ARM64或动力性能验证。

## 修复后复核（同日，生产差异由主代理更新）

主代理将首因记录移动到 `reject(...)` 之后，并限定 `phase_ == Phase::Safety`；没有改变 `reject` 的原控制流程。此前未解锁、可重新等待的临时拒绝现在不占用任务锁存首因；已进入Safety的后续事件仍走原有顶部恢复分支，不会覆写第一次记录。

独立重新编译执行前述失败复现，本次退出0：

```text
early phase=0 mask=0
fault phase=19 mask=2 stamp=3851000 expected_failsafe_bit=2 current_status=3851000
```

`native/tests/test_20261116_diagnostics.cpp` 已增加相同启动恢复路径；以 C++17/O2/Wall/Wextra/Werror/pedantic 独立编译运行退出0。`git diff --check` 仍退出0。

**前述确认 P2 已修复，本轮复核未发现剩余阻断项。** 初始失败和修复后结果都保留，不能将前述失败误读成最终仍未解决的状态。非阻断的range差分区间说明仍适用。
