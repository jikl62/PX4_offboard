# Stage 1 离线复现独立复核（2026-11-16）

## 结论

本次复核未发现阻断这份离线复现结论的错误。独立编译和执行成功，447 行 JSONL 与已有 `evidence/reproduction.jsonl` 逐字节一致，stderr 为 `STAGE1_REPRO_PASS`。这只证明合成输入下真实 Runtime/Core 和本地假 sink 的确定行为，不证明现场故障原因或真实飞行稳定。

## 参数核对

从 `attitude_ros_parameters.hpp` 独立解析生产参数名到成员的映射，再与 `evidence/runtime_20261114.yaml` 和 `repro/captured_config.hpp` 比较：64 项已出现的姿态控制参数全部匹配，未发现错配或数值差异。`profile`、runtime mode、entry version 和 `flight_enabled` 也与板端采集值一致。visual=false、arm=false、target=1.5、RC slot=1，推力 .50/.04/.12/.80/.40 没有被夹具覆盖。未出现在板端 YAML 的视觉参数仍用生产默认值。`fc01_config_path/source_path` 等来源元数据未写入夹具，不影响本次控制计算；夹具不等于完整 ROS 参数加载测试。Core 构造时把 height_lower_margin 传给其内部垂向控制器，故 captured_config 没单独写 vertical.ground_lower_margin 不会改变本次 Core 使用值。

## 断言和观测能证明什么

- normal_1p5：合成人工 RC 序列经过 ARMED_GROUND → THRUST_RAMP → ASCEND → HEIGHT_HOLD，最终 t=14.55 s。未生成自动 Arm/Offboard 命令。这是程序状态门测试，不是动力学仿真。
- active_reset_same_quaternion：保持姿态四元数不变，仅递增 quat_reset，t=14.60 s 进入 SAFETY，首因 ACTIVE_QUATERNION_RESET。心跳/目标假 sink 累计数都保持 291；之后 20 个合成健康控制周期仍不增加，z0 不重建。这里的“健康”指除 reset 事实外持续新鲜、有效的合成反馈，绝不是恢复动作后的硬件反馈。
- landed_blocks_liftoff：新高度/速度样本存在但 landed 保持 true，0.6 s 观察仍处 THRUST_RAMP，不能正式确认离地，且本地发布计划保留。该有限窗不能证明 landed 永久不变时一直停在 Ramp；生产代码另有超时恢复。
- cross_height_without_settling：合成轨迹穿过 1.5 m 后停在 1.70 m，超过 .15 m 高度容差，最终 t=15.85 s 进入 DESCENT，原因 ASCENT_TIMEOUT_SUPERVISED_RECOVERY；当轮仍保留发布计划。这个用例证明“穿过目标不等于满足稳定窗”，没有单独隔离速度容差条件，不应写成“已验证只有速度过大导致该次超时”。
- raw freshness：`p.received==0` 直接证明重复 raw 样本不会因 envelope 更新而修改接收进展时刻；后续 fresh=false 同时受到 raw 源年龄与主机进展年龄影响，不能用这个布尔单独分离两种拒绝来源。raw 回退后即使给新样本，regressed 保持且 fresh=false。

## 源码交叉核对

- `attitude_reference.hpp:37-43`：先归一化姿态；active 且 reset counter 不同，直接返回 ActiveReset，不要求四元数有可见变化。
- `attitude_altitude_core.hpp:1283-1287`：ActiveReset → fault(ACTIVE_QUATERNION_RESET) → 返回本轮默认空计划。
- 同文件 `2690-2704`：fault 写首因/最新原因并进入 Safety；`1165-1167` 后续 Safety 步只观察恢复地面证据并返回空计划。恢复仍需单独协议和严格地面证据，不能概括为“永远不能恢复”。
- `runtime.hpp:1084-1090`：调用该 Core，随后事务跟踪使用其空计划。没有替补姿态心跳的分支。
- `fc01_system_node.cpp:474-475, 500`：每轮先清 last_attitude_output，只有 `output.attitude.publish` 才进入姿态目标/心跳配对 sink；本轮空计划不会执行该发布块。通用路线发布在 645 行后另有条件，但当前 attitude_altitude profile 和默认空 flight output 不触发。
- `fc01_system_node.cpp:521-529`：生产本地 sink 返回后才记提交；这仍不证明 PX4 收到/执行。夹具用 `publish_attitude_at_boundary` 和固定上下文，生产用 `publish_attitude_resampled` 重采上下文；本测试没有覆盖 ROS 调度、DDS、消息队列和非原子发布之间新增事件。

## 独立复跑命令

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
g++ -std=c++17 -O2 -Wall -Wextra -I native/src/jikl_fc01_system/include -I native/tests docs/investigations/20261116_stage1/repro/reproduce.cpp -o /tmp/fc01-stage1-independent-repro
/tmp/fc01-stage1-independent-repro > /tmp/fc01-stage1-independent-repro.jsonl 2> /tmp/fc01-stage1-independent-repro.stderr
cmp docs/investigations/20261116_stage1/evidence/reproduction.jsonl /tmp/fc01-stage1-independent-repro.jsonl
cat /tmp/fc01-stage1-independent-repro.stderr
```

以上命令退出 0，编译无诊断。未修改生产文件、配置、Git 提交或硬件状态。
