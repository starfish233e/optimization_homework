# 优化理论及应用：第三章作业

本项目整理最优控制与动态规划的第三章作业题目、参考资料，以及第 3-3 题的 MATLAB 实现和计算结果。

## 文件说明

- [作业题目与索引](Homework%20for%20Chap3-2026.md)：六道作业题的 Markdown 整理版。
- [作业原始 PDF](Homework%20for%20Chap3-2026.pdf)。
- [第 3-3 题解答](output/q3/solution_q3.md)：推导、数值结果及增益变化解释。
- [MATLAB 脚本](output/q3/chapter3_q3.m)：有限时域最优控制计算、数值验证及图表生成。
- `output/q3/`：CSV 数据、PNG 图表及 MATLAB 结果文件。
- 根目录中的其余 PDF：第三章教材和讲义参考资料。

## 运行第 3-3 题

在项目根目录打开 MATLAB，执行：

```matlab
run('output/q3/chapter3_q3.m')
```

脚本不需要 Control System Toolbox，结果保存在 `output/q3/`。脚本包含 Riccati 方程残差、闭环稳定性、代价一致性及独立短时域二次规划验证。

临时渲染文件、本地 MATLAB 配置与运行日志由 `.gitignore` 排除。
