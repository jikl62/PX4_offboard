# FC01 预编译 ARM64 安装

适用：Orange Pi Zero 3W，Ubuntu22.04 ARM64，已有ROS2 Humble和FC01通信环境。本包无需板端编译或联网。设备上以原 `orangepi` 普通用户执行；只在安装器请求时输入sudo密码，不能sudo整个安装器。

把两个交付文件从U盘复制到桌面。拆桨、未解锁，机体接地静止；开启原通信/PX4反馈和RC，解锁/kill开关OFF，导航开关离开导航档。原服务需处于active并有一个控制节点；安装器会实时检查原监督器状态，不使用过期证据。

```bash
cd "$HOME/Desktop"
bash install_fc01.sh ./FC01_OrangePi_20261116_stage3_arm64.tar.gz
```

安装器固定校验相配包SHA256，然后校验Ubuntu/ARM64/Humble、全部文件摘要、ELF/动态库和消息类型支持。安装到 `$HOME/FC01_versions/20261116_stage3`。在原服务上添加单独drop-in切换，原unit和原目录保留；继承实际runtime全部控制参数，只改变版本/路径元数据。目标高度继承现场实际值，绝不强制重置1.5m。视觉保持关闭、人工RC解锁。只支持原有domain0/network DDS和 `/dev/ttyS2`115200通信合同；其他设置会拒绝而不修改。

成功打印 entry、backup 和 `FC01_ROLLBACK_COMMAND`；请保存该backup。再次执行相同安装命令会核对已运行的本版本，保持控制器PID和原回退点。

状态检查：

```bash
bash "$HOME/Desktop/install_fc01.sh" --status
systemctl --no-pager show fc01-navigation.service -p ActiveState -p MainPID -p ExecStart -p WorkingDirectory -p DropInPaths
python3 "$HOME/FC01_versions/20261116_stage3/navigation.py" status
pgrep -af '[f]c01_system_node'
```

查看连续状态（Ctrl+C只退出查看）：

```bash
python3 "$HOME/FC01_versions/20261116_stage3/navigation.py" monitor
```

回退可直接读取该版本安装记录中的原备份路径：

```bash
FC01_BACKUP=$(/usr/bin/python3 -c 'import json; from pathlib import Path; print(json.loads((Path.home()/"FC01_versions/20261116_stage3/deployment/INSTALL_STATE.json").read_text())["backup"])') &&
bash "$HOME/Desktop/install_fc01.sh" --rollback "$FC01_BACKUP"
```

回退同样要求安全地面；保留新版本目录与证据，恢复原drop-in状态和runtime，并核对旧服务、实际控制器及参数。失败时安装器尝试恢复原版；若无法安全退出或检测外部编辑，会明确报告 `manual_recovery_required` 和备份路径，不强杀、不覆盖他人修改。

重开已有服务只使用原命令 `sudo systemctl start fc01-navigation.service`；启动服务不会自动解锁。本文安装/状态/回退命令的实际验证环境及限制见最终报告，香橙派实机安装由现场人员执行。
