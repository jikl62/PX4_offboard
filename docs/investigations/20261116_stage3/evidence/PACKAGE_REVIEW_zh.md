# 阶段3归档包补充复核

复核当前 `docs/investigations/20261116_stage3`，测试提交为 `ce931551a6f6bc39e2b50a5377d70608882650f7`。本复核未修改仓库、未启动ROS节点或硬件。

结论：未发现归档完整性或报告结论的阻断问题。

- 从stage3目录执行 `sha256sum -c SHA256SUMS`，全部通过。
- 逐个读取3个tar归档全部266个文件成员：88个ROS原始文件、34个回归trace、144个早期尝试文件。归档SHA256、文件数与ARCHIVES.json一致，无解压读取或长度错误。
- ROS归档内11个result.json逐字节匹配evidence/scenarios下独立结果副本。
- SCENARIOS共11项，ATTEMPTS共29项，其中6项FAIL；报告未隐去早期失败。
- 报告正确区分replay预设零误差与plant模型结果，保留plant下降进入RECOVERY_DESCENT、10s下降窗口、约2s定高观察、XY固定、实际测距与PX4失联未实测等限制。
- 报告/命令明确未部署，保留新发行/ARM64身份核验要求，未将旧20261112.1身份当作阶段2修复版本。
- 在命令文档采用的清洁环境，仅source ROS Humble和px4_msgs后，rclpy/yaml/px4消息可导入；ldd可解析节点的px4_msgs与本包typesupport库，无not found。未启动节点。

小项：evidence/INDEPENDENT_REVIEW_zh.md首段仍引用`/tmp/fc01-stage3-independent-review.json`。相同JSON已保存为相邻`INDEPENDENT_REVIEW.json`，建议链接改为该归档内文件后重新生成SHA256SUMS。这不影响已有原始证据真实性。
