# ARM64 systemd 虚拟机实际安装验收

最终安装包 SHA256：`3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123`。
包内源码提交：`56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a`；控制逻辑候选：`adc55d0b7c67e80bf9d77b1082208c898f95b756`。

环境为 x86_64 开发机上的 QEMU system-aarch64 6.2 TCG、cortex-a72 2核/2GiB，guest Ubuntu 22.04.5 ARM64、generic 5.15.0-203、systemd 249 PID1、ROS 2 Humble ARM64。真实 `uname -m=aarch64`、`systemctl is-system-running=running`，并非 proot 或模拟 systemctl。没有连接 Orange Pi、PX4 或电机。

原版测试服务用阶段3生产 `navigation_boot.sh`、`navigation_service.py` 和 standby Supervisor，unit由原工程render_unit生成。原版与候选使用同一批从阶段3原生源码编出的真实 ARM64 ELF；因此回退验证的是版本路径、服务与参数事务，不声称对照了历史设备旧二进制行为。测试目标高度刻意设为1.23m、导航RC档位1，以验证安装器继承生效配置，未重置为默认值。

通信使用直接DDS合成的PX4未解锁、接地、静止反馈，以及独立ARM进程持有PTY模拟 `/dev/ttyS2`。该进程名为MicroXRCEAgent但不实现XRCE协议；本验收不证明实际UART、Agent、PX4或MTF-02P链路。所有地面反馈arm_switch=3、kill_switch=3、mode_slot=6，未请求解锁或进入Offboard。

实际结果：

| 场景 | 真实结果 | 证据 |
|---|---|---|
| 最终包安装 | 成功；服务指向独立版本目录；唯一候选ARM控制器PID4623 | actual-install-7.log、installed-final-state.json |
| 重复同包安装 | already_active=true；同一备份；控制器仍PID4623，未重启 | actual-repeat-install-final.log、repeated-final-state.json |
| 手动回退 | 成功；旧入口与唯一旧控制器PID5339；原runtime逐字节恢复 | actual-manual-rollback-final.log、manual-rollback-final-state.json |
| 新版启动超时自动回退 | 旧30s验收在TCG下超时后恢复旧版，真实路径/参数/DDS/地面验收通过；未强杀 | actual-install-6.log、install6-automatic-rollback.log、automatic-rollback-integrity.log |
| 最终源码ARM64测试 | 35安装器 + 6服务 + 2打包器，共43通过，17.710s | final-arm64-unit-tests.log |
| 地面输出检查 | 42083条设定值全部零推力、42083条心跳、无VehicleCommand | final-evidence/wire_summary.json、压缩原始DDS输出 |

安装、重复安装和回退均保持1.23m目标、RC档位1、视觉关闭、程序解锁关闭及全部推力参数。重复安装runtime完全相同。安装前后只改变 `fc01_entry_version` 和 `fc01_config_source_path` 两个身份字段；`fc01_config_path` 原本即为相同运行路径。主unit从未改写；通过专属drop-in切换，回退后drop-in消失。

真实环境揭示并修复了三个发行阻断：大型manifest单argv触发E2BIG、WorkingDirectory错误双引号导致systemd拒绝、完整预检实测46.377s超过原30s安装启动等待。另调整只读查询和快照读取顺序，保留原0.25s快照窗口。所有修复均在安装器，未改变飞行反馈TTL或控制逻辑。失败迭代日志完整保留。

注意原始心跳日志包含刻意停止服务和回退过程，最大间隔78.71s不能当作飞行心跳性能指标。最终硬件环境仍须用户拆桨、未解锁安装后检查真实UART/DDS、PX4反馈、调度与实际服务设置。本机没有执行板端安装或试飞。

最终用户命令补验：实际执行生成安装器 `--status` 和包内 `python3 navigation.py status` 均退出0，显示原版已恢复且READY、目标1.23m。按照INSTALL_STATE取backup再次请求已经完成的回退，安装器退出1并明确拒绝覆盖（预期保护）。原QEMU进程在前一轮交付结束后已退出，本补验重新启动同一guest磁盘并恢复仅地面夹具；因此补验的PID/boot/Agent不同于上表安装事务，未将重启后Agent身份冒称为原PID671。原事务同一次boot中的Agent身份始终保持。见user-status-and-repeat-rollback.log、post-restart-fixture.log。
