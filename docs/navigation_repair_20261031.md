# 2026-10-31 导航起飞链路修复记录

本次修改直接落在 `FC01_OrangePi_Navigation_20261028` 工程。修改前源码备份为 `/home/jikl62/FC01_OrangePi_Navigation_source_before_20261031.tar.gz`（不含 build/install、编辑器目录和缓存）。工程没有 Git 元数据。

## 已确认的原因与改动

- Python 配置固定输出档位 6，C++ 同样只接受 6。`navigation_settings.json` 新增 `navigation_mode_slot`，本工程按既有飞控记录设为 1；Python 严格校验整数 1–6，C++ 接受对应范围，最终授权仍核对实际档位与实际 Offboard。
- 已准备后，armed 反馈先到、RC 解锁开关后到时，旧状态机停止准备发布；实际 Offboard 先于遥控档位到达也会被拒绝。本次使用有界的零推力等待许可，最终发布时重新核对当前反馈；没有完整手动授权时不能产生非零推力。固定的证据期限在接受互补证据之前检查，避免超时当周期被迟到消息绕过。
- 后台重试丢弃飞机状态；状态窗口又只检查快照写入时间。现在保留最后反馈的原始接收年龄，过期/退出/重连时标为历史，不能因后台更新快照而变为当前反馈。
- 稳态反馈曾每 5 秒更新一次快照，而界面要求反馈年龄不超过 3 秒；现在每秒刷新稳态快照，日志继续按状态去重，陈旧阶段标题改为 STALE。
- IPC 正常 EOF、残缺 JSON 和控制器退出原先混在一起。现在分别记录通道错误分类与原生进程退出结果，完整消息之后出现残缺尾部也保留完整消息。

没有调整悬停推力、起飞余量、PID、倾角、高度、反馈一致性保护或任务时限。程序继续仅接受物理解锁和导航触发，不主动发送 ARM/进入 Offboard 命令。

## 核对状态

当前反馈查看 `feedback_current`、`state_age_seconds` 和原生 `status_fresh/local_fresh`，历史状态只能用于解释最后一次观测。通道结束查看 `ipc_error`；进程退出查看 `controller_returncode/controller_exit_code/controller_exit_signal`。

原生状态的 `rc_navigation_mode_slot` 为期望档位，`rc_mode_slot` 为实际观测档位，`nav_state` 为实际飞控模式；只有档位对应且 Offboard 确认才加入起飞流程。高度未知时查看 `height_reference_ready`、`height_unavailable_reason`、`status_age_s/local_age_s`，不要用零高度代替未知。

## 验证

修改前完整基线：`python3 -m unittest discover -s tests -v`，96 项通过。新增回归已在修改前复现档位拒绝、独立消息乱序和历史状态覆盖等缺陷。

最终完整回归：`python3 -m unittest discover -s tests -v`，**115 项通过，44.789 秒**。其中包含 **54 个真实 C++ 回归程序**及实际 ROS 节点/配置检查器的 API 编译夹具。另行执行 `python3 tests/run_native_cpp.py` 得到 `NATIVE_CPP_PASS 54 executables`，节点 API 夹具也通过。

独立审查发现的三项问题均已修复并增加先失败后通过的回归：

- 实际 Offboard 与导航档位两种消息顺序各延迟 500ms，均连续提交 10 对零推力目标/心跳；永久错档、撤销、失效源、非零注入与配对超时拒绝。
- 2.5 秒证据期限已过、在 2.55 秒才补齐手动解锁证据时，必须在授予证明之前拒绝；导航证据也先检查期限。
- 持续稳定阶段会在界面新鲜度期限内刷新快照；静默、退出、重试和旧进程状态不能冒充当前反馈，过期阶段标题显示 STALE。

离线回放和 API 编译夹具验证的是软件行为，无法证明实机 DDS 回复或实际离地。

最终真实 Humble 重建使用本机原 FC01 工程的 `install_r632` 消息 SDK，`fc01_system_node` 和 `fc01_config_check` 均编译、链接通过，实际修复 YAML 加载通过。电脑通用 `ros2_px4_ws` 的 `px4_msgs` 缺少原工程要求的 `VehicleAttitudeSetpoint.reset_integral/fw_control_yaw_wheel`，因此该环境的首次控制器编译失败；切换配套 FC01 SDK 后通过。没有通过删除这些字段来迁就不匹配的 SDK。

实际 rclcpp 配置检查器已用真实 YAML 验证 5 种情况：档位 1、6 通过，档位 0、7 和 `arm_enabled=true` 拒绝。本机编译架构为 x86_64；香橙派仍须执行 ARM64 构建。

`native/config/attitude_altitude.yaml`、垂直控制器、垂直反馈保护、ROS 参数映射及安装流程文件已与修改前备份逐字节核对，均保持原内容。

验证日志：`/tmp/fc01_navigation_final_tests_20261031.log`、`/tmp/fc01_navigation_fc01sdk_final_build_20261031.log`。完整源码安装包位于 `dist/FC01_OrangePi_Navigation_20261031_fixed.sh`，只包含源码；构建后检查 shell 语法，并在临时目录执行两次 `--install-only` 解压、逐文件对比。

## 部署

把修改后的源码工程复制到香橙派的新目录，在飞机未解锁、接地静止时，从该目录运行：

```bash
bash navigation.sh install
```

安装会在香橙派本机重新构建 ARM64 控制器、验证实际 YAML，并被动确认地面反馈后切换托管服务。本机 x86_64 离线测试产物不用于飞控运行。

如果使用生成的源码安装包，可指定新目录，避免与旧工程内容冲突：

```bash
FC01_NAVIGATION_INSTALL_DIR="$HOME/FC01_OrangePi_Navigation_20261031_fixed" \
  bash ~/Desktop/FC01_OrangePi_Navigation_20261031_fixed.sh
```

安装后仍需核对 service 的 WorkingDirectory、控制器二进制路径、`~/FC01_navigation/runtime.yaml` 中档位和运行反馈。DDS `replies=0/sessions=0` 属于另一条待定位链路，本补丁不据此宣称通信和自主起飞已通过验收。
