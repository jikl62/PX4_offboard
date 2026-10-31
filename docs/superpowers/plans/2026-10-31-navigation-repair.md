# 导航起飞阻断修复实施记录

**依据：** 用户提供的部署源码审查上下文，并授权直接在本工程修改。
**目标：** 修复已确认的软件阻断，完成本机离线验证；实机 DDS 零回复和实际起飞仍需部署后证据。
**结构：** 保持 Python 配置/监督与 C++ 控制/最终发布的分工，使用现有 unittest 与 C++ 回放。
**工作方式：** 原目录直接修改；目录没有 Git，原始源码备份为 `/home/jikl62/FC01_OrangePi_Navigation_source_before_20261031.tar.gz`（排除编译缓存、编辑器文件）。

## 约束

- 手动 CH5 解锁、CH6 选择导航；程序不发送 ARM 或 Offboard 进入命令。
- 沿用现有推力、PID、反馈保护和时限。未授权只允许零推力准备输出。
- 档位采用飞控报告的 mode_slot（1–6），不把 CH6 物理通道号当成档位。
- 历史反馈不得被显示或使用为当前新鲜反馈。

## 检查重点

1. 档位 1 与 6 均能走通授权，错误/越界档位不能启动。
2. 已解锁状态与 RC 开关分别先到，准备发布必须连续；缺少匹配证据必须超时拒绝。
3. 准备时非零推力、旧来源事件、错误解锁原因、地面变化、Kill 等仍拒绝。
4. 完整 IPC 消息后紧接残缺消息/EOF，完整状态仍可保留，关闭原因可区分。
5. 重试、控制器退出、后台快照更新不能刷新历史飞机状态。

## 任务

- [x] 配置贯通：`navigation_settings.json` 可设 `navigation_mode_slot`，本工程设置为既有飞控记录的 1；Python 严格校验整数 1–6，缺省保持旧配置兼容；C++ 接受范围 1–6。先增加参数生成/真实 C++ 校验回归，确认失败再修复。
- [x] 准备输出：修复 `rc_navigation.hpp` 与 `publication_guard.hpp` 的独立消息汇合；在 `test_rc_navigation.cpp` 增加延迟、连续提交和拒绝路径回归，先失败后通过。
- [x] 后台诊断：修复 `launcher.py` IPC 分类、`standby.py` 退出与历史状态保留、`navigation.py` 新鲜/历史显示，使用真实 socketpair/子进程回归先失败后修复。
- [x] 可观察性与交付：补充期望档位、状态年龄和高度未知原因，更新使用说明；运行完整 Python、C++ 和 ROS API 编译夹具，构建并检查源码安装包，由独立审查确认改动。

验证主命令：`python3 -m unittest discover -s tests -v`；`python3 tests/run_native_cpp.py`；`python3 native/tests/adapter_compile_check.py`；`python3 build_navigation_bundle.py dist/FC01_OrangePi_Navigation_20261031_fixed.sh`；`bash -n dist/FC01_OrangePi_Navigation_20261031_fixed.sh`。

最终验证：115 项完整测试通过（含 54 个 C++ 回归程序），实际 Humble/FC01 SDK 构建与 YAML 加载通过。独立审查提出的 Offboard 乱序、迟到证据绕过期限、快照节流三项 Important 均已回归修复。
