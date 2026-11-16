# 实际 VM 命令与边界

测试通过开发机本地 SSH 转发进入 QEMU guest，guest root 仅用于制作隔离测试夹具；安装器始终作为 uid1000 orangepi 执行。命令不针对真实香橙派。

最终安装：
```bash
runuser -u orangepi -- bash /home/orangepi/install_fc01.sh /home/orangepi/FC01_OrangePi_20261116_stage3_arm64.tar.gz
```
重复安装：同一命令，核对 already_active=true 且 native PID 不变。

状态检查：
```bash
systemctl show fc01-navigation.service -p MainPID -p ActiveState -p WorkingDirectory -p ExecStart -p DropInPaths
python3 /root/vm-state-capture.py
fuser /dev/ttyS2
```

回退命令使用实际安装结果中的 backup 字段；本次真实路径见下文。

夹具始终未解锁、接地、静止；directDDS发布真实Humble类型，Agent为仅持有PTY的小程序。无UART、真实XRCE、PX4、MTF-02P或OrangePi硬件。安装前置完整性/动态库/type support/nativeconfigchecker、真实proc/systemd/fuser身份和事务恢复实际执行。

实际手动回退（已成功，原参数逐字节恢复）：
```bash
runuser -u orangepi -- bash /home/orangepi/install_fc01.sh --rollback /home/orangepi/FC01_navigation/prebuilt-backups/20261116T135220_254940Z-3669
```
最终包：SHA256 `3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123`，source_commit `56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a`，control_candidate_commit `adc55d0b7c67e80bf9d77b1082208c898f95b756`。

最终用户入口补验在同一guest磁盘重新引导后执行，普通orangepi用户命令：

```bash
bash /home/orangepi/install_fc01.sh --status
python3 /home/orangepi/FC01_versions/20261116_stage3/navigation.py status
```

二者退出0，显示已恢复的旧版控制器READY、目标1.23m。原始输出见 `user-status-and-repeat-rollback.log`。重新引导后的PID/boot不属于前述安装事务，不作同PID连续运行证据。

以下文档表达式实际读出了与成功手动回退相同的备份路径：

```bash
FC01_BACKUP=$(/usr/bin/python3 -c 'import json; from pathlib import Path; print(json.loads((Path.home()/"FC01_versions/20261116_stage3/deployment/INSTALL_STATE.json").read_text())["backup"])')
bash /home/orangepi/install_fc01.sh --rollback "$FC01_BACKUP"
```

因该回退此前已完成，第二次请求退出1并提示“当前服务切换记录不匹配，拒绝覆盖”，属预期保护。成功的首次回退证据仍为 `actual-manual-rollback-final.log`。
