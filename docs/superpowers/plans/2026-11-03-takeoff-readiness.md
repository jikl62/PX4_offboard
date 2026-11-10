# 起飞与准备修复执行计划

依据 [设计](../specs/2026-11-03-takeoff-readiness-design.md)。按用户已授权的直接实现方式推进，并行处理互不重叠文件；每项先复现问题。基线回归：160/160，28.539 s，日志 `/tmp/fc01_1103_baseline_tests.log`。

1. **配置一致性**：集中 Python 默认高度，贯通 C++ 参数、设置、导出、CLI、回放和打包；以生成真实 YAML→原生 Config 的测试验证 1.5 m，保留非高度的 0.5。记录 SDK 基础配置与包内基础配置区别。
2. **起飞证据/Core**：提取独立证据类，RED→GREEN 覆盖异步正反顺序、单点/冻结/矛盾 landed、重置、到高前无导航；不修改通用 touchdown 守卫。暴露 `takeoff_stage/liftoff_confirmed/target_height_settled/navigation_allowed`。RC 重准备接口保持 `rc_restart_allowed(Feedback)`。
3. **服务与任务恢复**：Runtime 增加 `reprepare_rc(Snapshot,double)`，保留 ACK 历史；节点不再因正常复位退出。测试第二轮新授权、陈旧 ACK、通用故障/发布故障/活动输出拒绝。Python preflight/validate 使用实际 controller。
4. **观测与结构**：节点使用 Core 唯一高度基准，提取重复的状态/推力字段，记录反馈年龄和循环耗时；已有异步日志/非阻塞 IPC 不盲目重写。测试直接覆盖提取与新状态，不制造镜像实现的测试。
5. **通信证据**：整理已有限流/固件/ULog证据和 UART6 未知项；只读 `navigation_collect.py` 记录部署来源与可选被动 ROS 统计，缺依赖或超时明确 unknown，不触发服务或飞行动作。
6. **收尾**：必要完整回归和实际 ROS 构建/YAML检查、离线 1.5 m 升降及导航回放；新上下文独立审查，修复其确凿问题。按行为/结构分别交付、打包并验证归档，不切实机服务、不合并主线。

审查重点：异步汇合不能消耗/重标记旧检测事件；不因未确认离地误判已接地；第二轮不得复用视觉指令或旧命令 ACK；1.5 m 轨迹在原 10 s 上升时限内是否可由候选模型验证；源时间、PX4 boot时间与本地 monotonic 保持区分；采集工具绝不启动生产节点。
