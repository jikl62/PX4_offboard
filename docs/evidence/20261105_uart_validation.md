# 2026-11-05 双串口审计验证记录（历史记录）

> **2026-11-06 状态：** 本记录保留 11/05 的 206 项回归基线和当时的主机 smoke 结果；它不再是当前物理拓扑结论。用户最新物理记录与 FC01 `/dev/ttyS3` → 本地源码 UART4 的运行证据冲突，请以 [20261106_uart_mapping_audit.json](20261106_uart_mapping_audit.json)、[20261106_uart_validation.md](20261106_uart_validation.md) 和[双串口映射审计](../FC01_DUAL_UART_20261105.md) 为准。

本记录使用用户环境日期2026-11-05。开发机为x86；没有连接香橙派或FC01。本轮仅增加被动采集工具、测试与文档；没有部署或启用第二链路。

| 检查 | 实际结果 | 可证明的范围 |
|---|---|---|
| `python3 -m unittest discover -s tests -v` | **206/206通过，30.352s**；日志 `/tmp/fc01_20261105_uart_regression.log` | 原182项与新增24项通过；其中原生回归入口继续执行60个C++测试程序，另有适配器编译检查。不是实机串口或飞行验证 |
| 新增 `tests/test_uart_preflight.py` | **24项通过**，包含真实x86 CLI执行 | 被动检查器的边界、报告、错误分支与模拟设备树一致性 |
| `python3 -m py_compile uart_preflight.py tests/test_uart_preflight.py` | 通过 | Python语法与字节码编译 |
| 固件来源证据校验 | [12个关键源文件/生成文件](20261105_uart_source_audit.json)的SHA256重新计算全部匹配 | 本地所审源码没有在核查过程中变化；不证明已刷固件身份 |
| 开发机实际采集 | [主机结果摘要](20261105_uart_host_smoke.json)：`unknown`、`NOT_CONFIRMED_ON_ORANGEPI`、`NOT_PERFORMED` | 能在非香橙派主机运行且不误报硬件可用；原始本机报告在 `/tmp/fc01_uart_host_1105_02h8y29_/report` |
| 修改范围 | 生产入口、控制器、YAML/JSON配置和服务未修改 | 原有fc01_config_check.cpp空白差异是用户预先修改，保留且不纳入本轮提交 |

24项针对性测试覆盖：正常模拟证据、非字符/缺设备、缺DT、UART或父节点disabled、缺状态/权限不足、alias不能冒充绑定、driver缺失、pinctrl缺项、fuser退出码与失败/getty状态、多来源console/DT stdout/earlycon、serial6与uart6/PE0/PE1一致性、不同函数引脚不能拼凑、同设备别名与主次设备号重合、异常symlink与有界读取、不打开设备/不改服务、虚拟rootfs禁用真实命令、输出目录拒绝覆盖、非法设备参数/命令白名单/超时，以及真实x86 smoke。

单独复现新增检查器测试：

```bash
python3 -m unittest discover -s tests -p 'test_uart_preflight.py' -v
```

设备核查使用：

```bash
python3 uart_preflight.py --output "$HOME/FC01_UART6_preflight_20261105_01"
```

待设备验证项目：OrangePi当前镜像/板型/驱动/DT/pinmux与ttyS6、全局占用与电平；FC01 UART4映射及所有者、当前固件身份；实际双向字节、排队/CRC/丢帧和源样本推进。第二传输协议、话题分区、运行健康监控属于[后续源码方案](../FC01_DUAL_UART_20261105.md)，本轮未实现、未构建新PX4镜像、未启动第二Agent、未验收双链路容量或动力表现。

合并时只纳入本轮审计工具/测试/文档提交，基线为`6c5b185`。无需重启或切换现用导航服务；检查器必须手动运行，不加入飞行自启入口。
