# 视觉导航实现计划

**Goal:** 在当前导航修复快照上增加真正参与姿态/推力计算的视觉运动通路，默认关闭，开发分支离线交付。
**Spec:** `docs/superpowers/specs/2026-10-31-vision-navigation-design.md`
**Architecture:** 规范化状态与目标→纯 C++ 外环→原 Core/权限/最终双 sink；ROS adapter 和无硬件 preview 分离。
**Tech Stack:** C++17、ROS2 Humble、自定义 ROS 消息、Python3、现有 unittest/host C++ fixtures。

## 全局约束与审查重点

保留导航修复、原 PID/推力/保护与手动解锁；默认不启用新控制，不变更服务、飞控或硬件。未确认真实相机接口不得伪造有效字段。重点审查状态/目标分别过期、坐标方向、重置锁存、下调高度限速、降落 yaw 连续、final TTL 竞态和未授权倾斜拒绝。

## 任务

- [x] 核对源码/无 Git 状态；保存修复快照基线 `6b440ab`，建立开发分支。旧完整回归 115 项通过。
- [x] 原生数据合同与运动外环：新增 `vision_navigation.hpp` 与行为回归，先 RED 后实现；状态进入 XY 闭环和 z 目标转换。
- [x] ROS 内部消息与外部适配：typed state/command/health/preview、自定义接口生成、可配置 topics/QoS、Odometry+health 的明确转换和字段缺失拒绝，先测试后实现。
- [x] 原控制链集成：Config/Feedback/Runtime/参数/node 输入，HeightHold 运动、原生降落、提交 yaw 与 final guard；完整授权和拒绝回归。
- [x] 无硬件模拟/回放：走 Runtime+双 sink 的可运行工具、JSONL、确定性闭环场景、非法/过期/中断/切换回归。
- [x] 独立审查、完整回归、真实 Humble/SDK 构建、接口/运行/待确认/合并文档；提交开发分支，不切实机服务。

## 并行边界

纯控制头/其测试、ROS 接口/适配工具、离线 harness/用户文档互不修改同一文件；root 负责原 Core、Runtime、guard、node、参数集成。代理只做获分配文件；提交由 root 统一管理，避免共享 index 冲突。

完成记录：160 项完整回归与 56 个 C++ 回归通过；真实 Humble/原 FC01 SDK 消息生成、构建、安装、运行库检查、YAML 检查通过；真实 ROS 合成适配烟测通过。独立审查三项 finding 及上游多源/私有库问题已回归修复。设备飞行验收另行安排。
