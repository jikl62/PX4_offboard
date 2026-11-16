# ARM64预编译服务入口独立复核

2026-11-16。只读审查 `navigation_prebuilt_service.py`、`prebuilt_boot.sh`、`tests/test_navigation_prebuilt_service.py`，并沿真实 `navigation_service.check_controller`、`standby.supervise`、`launcher.preflight` 复查集成。

## 结论

未发现阻止当前预编译入口集成的确定性代码缺陷。此结论支持入口/配置合同与单元夹具范围，不能替代实际ARM64动态库检查或真实systemd生命周期验证。

- 明确指向包内 `install/lib/jikl_fc01_system/fc01_system_node`，不回落旧SDK控制器。
- 启动前核对发行清单、payload全部普通文件摘要、架构标识、`ldd`、打包消息Python来源/type support、真实C++配置检查器。没有把消息.msg存在等同类型支持完整。
- `snapshot_profile`深复制整个继承配置，只修改三项身份元数据；原生控制参数、目标、RC/视觉/动作门不被runtime_profile覆盖。额外key和其他node作用域也保留。
- 限定已验证的人工RC flight-test合同，不以默认值打开Arm/视觉或通用flight门；不兼容策略、非0DDS域、非固定串口/波特率拒绝，未静默“修正”。
- boot先去掉旧overlay路径，再source系统Humble和包内merged install；继承DDS/RMW键通过NUL数组export，无eval。普通过程的HOME/PATH及LD_PRELOAD等不从JSON重放。
- 复用原监督器，显式传新二进制和快照路径，`launcher.preflight`会消费该路径而非旧root/config；原Agent复用、唯一控制器、停止和安全退出机制保留。
- 每轮原生重启前重核payload、控制器和部署快照未变；不创建新的飞行授权API。
- `check`不启动控制器/Agent，不打开串口；C++配置检查器会建立参数检查ROS节点但不创建动力发布器、不spin。应如实称配置检查，不能说完全不创建ROS上下文。

## 实际复跑

```text
PYTHONPATH=tests:. python3 -m unittest test_navigation_service test_navigation_prebuilt_service -q
Ran 32 tests ... OK
bash -n prebuilt_boot.sh -> 0
python3 -m py_compile navigation_prebuilt_service.py -> 0
git diff --check -> 0
```

日志：`/tmp/fc01-prebuilt-service-independent-tests.log`。该32项包含现有26项和新增6项，不把子断言重复计数。测试显式注入ARM64架构/执行函数，伪ELF仅是单元夹具；此报告没有把它们当实际ARM二进制证据。未启动真实ROS控制器、Agent、systemctl或访问硬件。

## 交付需保持的限制

- 实际包仍需父任务在Ubuntu22.04 ARM64解释执行环境核对ELF/type support/ldd及可搬迁install，且检查没有混入宿主x86库。
- 固定domain0/UART2-115200与原Supervisor相符，但不兼容用户不同现网设置时应明确安装前拒绝，不能承诺支持任意DDS/Agent配置。
- 当前check并不核验串口/Agent实时占用、飞机状态；这些由安装事务/真实运行preflight分别处理，不应把包check成功写成可解锁或可飞。
- 未实际验证真systemd PID1；安装/重复安装/回退用fake system manager的结果必须和真实ARM64用户态检查分开报告。

未修改任何源码或提交；未扩大飞行行为。

## 安装事务集成补查（20:57起）

只读补查发现当前安装器的地面gate与真实正常READY不一致：`require_ground` 要求active=false且heartbeat_enabled=false，但Stage3真实ROS记录的RC_READY_WAIT_ARM为active=true、heartbeat_enabled=true、armed=false、landed/at_rest=true、nav_state15、精确零推力、解锁OFF/非导航档。复现文件 `/tmp/fc01-prebuilt-installer-ground-repro.py`、日志同名前缀.log。此问题会阻断正常安装前检查和新版本验收；已通知安装器作者和root，需以真实READY事实增加回归并最小调整安装维护gate，不能放开控制权。

此外旧Supervisor自建Agent在安全停止后可能由finish_agent终止，而安装器要求同PID Agent存活。简单fake注入可使其在stop之后拒绝切换；当前未改unit分支会restart旧服务，但该分支未等待旧实际控制器/反馈恢复就声称已恢复。真实旧服务Agent所有权需明确，最好安装前拒绝不支持的ownedAgent情形或实现安全交接与复验。该问题不是飞行算法缺陷。

当前运行参数的RC字段还应与live STATUS对照；仅比较目标高度不足以区分“运行中的有效RC”与后来被编辑的磁盘runtime。此项为审查建议，尚未完成故障复现。
