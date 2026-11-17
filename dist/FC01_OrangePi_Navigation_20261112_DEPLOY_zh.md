# FC01 20261112.1：香橙派安装与测试入口

将下面三份文件一起复制到香橙派的 `~/Desktop`：

- `FC01_OrangePi_Navigation_20261112_release.sh`
- `FC01_OrangePi_Navigation_20261112_release.sh.sha256`
- 本说明。

新安装目录：`~/FC01_OrangePi_Navigation_20261112_release`。保留旧目录、SDK 和服务备份。操作使用普通 `orangepi` 用户。安装包不自动启动服务；后续在线配置、启动安排在拆桨、未解锁、接地静止、RC 复位时进行。

## 1. 校验并仅解包

```bash
cd "$HOME/Desktop"
sha256sum -c FC01_OrangePi_Navigation_20261112_release.sh.sha256
bash ./FC01_OrangePi_Navigation_20261112_release.sh --install-only
```

应显示 SHA 校验成功和 `FC01_EXTRACT_ONLY_COMPLETE`。已有同名目录内容不同会拒绝覆盖；不要覆盖旧安装目录。`--extract-only` 是同义选项。不带选项会解包后执行离线构建。

## 2. 使用现有 SDK 做全新 ARM64 构建

```bash
FC01_RELEASE_DIR="$HOME/FC01_OrangePi_Navigation_20261112_release"
export FC01_PROJECT_ROOT="$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"
test -f "$FC01_PROJECT_ROOT/install_r632/local_setup.bash"
bash "$FC01_RELEASE_DIR/navigation_deploy.sh" offline
```

SDK 位置不同，只修改 `FC01_PROJECT_ROOT`；新控制器仍从本包源码构建。依赖现有 Ubuntu ARM64、Humble、px4_msgs、CMake/G++ 和 Agent 环境，不重新安装整套 ROS。保持此终端，后续命令会沿用 SDK 设置。

应出现 `FC01_ARM64_FRESH_BUILD_VERIFIED`（或完整证据校验后的 `FC01_ARM64_BUILD_REUSED`），最后为 `FC01_OFFLINE_READY`。此步骤不运行生产控制器或 Agent，不操作服务。可以只读核对产物：

```bash
readelf -h "$FC01_RELEASE_DIR/install/lib/jikl_fc01_system/fc01_system_node"
python3 - <<'PY'
import json
from pathlib import Path
p = Path.home() / 'FC01_OrangePi_Navigation_20261112_release'
r = json.loads((p / 'FC01_RELEASE.json').read_text())
b = json.loads((p / 'ARM64_BUILD_EVIDENCE.json').read_text())
print('发行版本:', r['version'])
print('源码提交:', r['source_commit'])
print('构建提交:', b['source_commit'])
assert r['source_commit'] == b['source_commit']
PY
```

ELF 的 `Machine` 必须为 `AArch64`，源码提交应为 `32990e599a68fde377daadbb89710b77fda1fb82`。自动离线校验还绑定依赖、文件摘要、工具链及安装树，不能仅凭文件名认定是新版。不要移动构建目录或单独复制 install。

## 3. 配置服务，保持停止

先只读查看旧服务：

```bash
systemctl --no-pager show fc01-navigation.service fc01-standby.service \
  -p Id -p ActiveState -p MainPID -p WorkingDirectory -p UnitFileState
```

现场确认可以停止后，停止已有导航服务：

```bash
sudo systemctl stop fc01-navigation.service
```

如果旧 `fc01-standby.service` 存在且仍运行/启用，另行正常停止并停用它。不要强杀控制器。配置入口要求服务已停止、没有残留控制器，以及仍有同用户 `/dev/ttyS2`、115200 的唯一 Agent。停止旧服务可能同时关闭它自建的 Agent；遇到这类情况，按包内 `docs/FC01_ORANGEPI_RELEASE_20261112_zh.md` 的前台 Agent 恢复步骤处理，不启动第二个串口占用者。

```bash
bash "$FC01_RELEASE_DIR/navigation_restore.sh" configure \
  --confirm-flight-test --boot-policy preserve
```

应显示 `FC01_SERVICE_NOT_STARTED` 和 `rollback=完整备份路径`，保存该路径。`preserve` 保留原开机策略；不会主动关闭原有自启动。该分步入口不替代完整 activate 事务的构建证据/启动后验收，所以必须先完成第 2 步且此后不修改源码或产物。

## 4. 人工启动并查看状态

```bash
sudo systemctl start fc01-navigation.service
systemctl --no-pager show fc01-navigation.service \
  -p ActiveState -p MainPID -p WorkingDirectory
bash "$FC01_RELEASE_DIR/navigation.sh" status
```

`WorkingDirectory` 应为新目录。状态入口继续显示模式、armed、目标/实际高度、推力、阶段、RC 和故障原因。版本也可以从已有状态快照读取：

```bash
python3 - <<'PY'
import json
from pathlib import Path
r = json.loads((Path.home() / 'FC01_navigation/status.json').read_text())
print('快照服务PID:', r.get('supervisor_pid'))
print('入口版本:', r.get('entry_version'))
print('原生配置版本:', r.get('state', {}).get('entry_version'))
print('配置路径:', r.get('config_path'))
PY
```

快照 PID 应与 systemd 的 MainPID 一致；同时核对 `status` 的新鲜度提示，历史快照不代表当前飞机状态。入口版本应为 `20261112.1`。C++ 内已有的 `FC01_RECOVERY_DIAGNOSTIC_V5_20261110` 是能力标记，不是源码提交；真正源码身份由发行与构建清单绑定。

需要完整事务切换和启动后自动验收时，可选择包内说明的 `navigation_deploy.sh activate --confirm-flight-test --boot-policy preserve` 路线；它会实际启动服务，不是“只配置”。不要把两条路线连续执行当成必需步骤。

## 5. 日志、报告和回滚

```bash
bash "$FC01_RELEASE_DIR/navigation.sh" logs
bash "$FC01_RELEASE_DIR/navigation.sh" report
```

测试后提供打印出的报告 ZIP、现场方式、遥控器操作顺序和发生时间。

回滚使用配置/激活实际打印的备份路径：

```bash
FC01_ROLLBACK_DIR="$HOME/FC01_navigation/service-backups/替换为实际备份目录"
bash "$FC01_RELEASE_DIR/navigation_restore.sh" rollback --backup "$FC01_ROLLBACK_DIR"
```

成功提示 `FC01_SERVICE_ROLLED_BACK_NOT_STARTED`，旧定义已恢复、服务保持停止。先核对身份，再由人决定启动。

## 本包保持的行为和验证边界

人工 RC 解锁、无视觉 1.5 m 起飞、高度闭环、接管、Kill、Failsafe、锁存及安全恢复代码未因本次发行修改；视觉仍关闭。推力参数保持 hover=0.50、reserve=0.04、min=0.12、max=0.80、slew=0.40。

开发机已完成独立解包、源码提交逐文件比对和全新 x86 Humble 构建。Orange Pi 的 ARM64 构建、UART/DDS、PX4 实际反馈及运行负载尚未实测；这些需要在目标板按上述步骤验证。包内不携带开发机 x86 ELF，不自动解锁或起飞。
