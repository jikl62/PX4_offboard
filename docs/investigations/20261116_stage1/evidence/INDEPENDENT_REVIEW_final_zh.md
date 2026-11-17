# 阶段 1 最终交接独立复核

日期：2026-11-16。审查对象：`docs/investigations/20261116_stage1/STAGE2_HANDOFF_zh.md`、最新复现代码及对应证据。

## 结论

未发现阻断交接结论的事实性错误。报告区分了最新版板端归档、旧版试飞、SITL、合成离线复现和当前现场未知条件，没有把不同试飞拼接成已证实因果链。

最终复现代码已由本独立审查进程重新编译执行：退出 0，449 行 JSONL 与已归档 `reproduction.jsonl` 逐字节一致。stderr 为最新 `STAGE1_REPRO_PASS`，包含后来增加的 queued failsafe 和 moving-stick 两例。因此除初版 447 行审查之外，最新 449 行驱动现在也已有独立复跑。未重跑本轮全部 8 个原有测试或 UBSan；对这些项目仅核验现有记录及普通/UBSan 输出哈希相同。

## 实际核验

- Git HEAD 为 `3677b9dd1f35e092e43501d26164c6263545c947`，与 BASELINE 一致；tracked staged/unstaged diff 均为空；BASELINE 列出的 13 项生产源码/配置哈希全部匹配。
- 两个 companion ZIP 的实际 SHA256 与归档索引相同，ZIP CRC 检查通过。最新报告的 94 项 source_sha256 与当前源码逐一独立校验一致。
- 最新 companion 的 journal 明确记录 09:30:06 等待导航、09:30:07 queued safety；详细日志起点更晚。报告没有从健康末帧反推此前没有故障，也没有把 09:30 问题定为唯一输入。
- 最新归档日志中 `.2/.1/current` 的快照数量为 233/232/92，phase_event 为 233/241/91，与报告一致。报告正确区分 phase_event 周期行与真实阶段变化。
- 扩大库存 30 个唯一 ULog 中只有一份包含 Position；该份内容元数据是 PX4_SITL，不是当前 MTF 实飞。报告没有把缺失的稳定 Position ULog 当作已验证证据。
- 垂向重放记录确为 13 段、5 段健康、8 段位置/导数一致性失败；报告明确未按控制阶段过滤、可能降采样，未宣称已经复现实飞误报。
- 源码确认 reset counter 即使 q 不变也能触发 ActiveReset；Core 返回空计划，后续 Safety 不自动恢复。HeightHold 稳定窗谓词与报告一致。
- 匹配 PX4 `offboardCheck.cpp:46–47` 直接以 COM_OF_LOSS_T 检查年龄；报告没有沿用“固定 500 ms 再等待一次”的错误描述。

## 小问题

P3 文档排版：交接报告第 103 行的表格单元格含未转义 `|vz|`，GitHub Markdown 会把它分成额外列。改成 `abs(vz)` 或转义两根竖线即可。只影响阅读，不影响证据结论。

建议把“后 2 s 增至 reserve”写成“后 2 s 由 hover 增至 hover+reserve”，避免初学者误读为最终推力 .04。上下文公式和参数表正确，此项是措辞改进。

## 最新驱动独立复跑命令

```bash
g++ -std=c++17 -O2 -Wall -Wextra \
  -I native/src/jikl_fc01_system/include -I native/tests \
  docs/investigations/20261116_stage1/repro/reproduce.cpp \
  -o /tmp/fc01-stage1-final-review-repro
/tmp/fc01-stage1-final-review-repro \
  > /tmp/fc01-stage1-final-review-repro.jsonl \
  2> /tmp/fc01-stage1-final-review-repro.stderr
cmp docs/investigations/20261116_stage1/evidence/reproduction.jsonl \
  /tmp/fc01-stage1-final-review-repro.jsonl
```

以上退出 0。普通与 UBSan 归档 JSONL SHA256 均为 `4df1ea6a215cc2ea8f4b0e16667d1e4a44e66487a01a106674d02cbdc68e6b1e`。

本复核未修改生产源码/配置、未访问硬件或启动服务、未创建 Git 提交。
