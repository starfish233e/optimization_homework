---
title: "第三章作业题目"
source_pdf: "Homework for Chap3-2026.pdf"
source_pages: 1
question_count: 6
question_ids: ["3-1", "3-2", "3-3", "3-4", "3-5", "3-6"]
language: "en"
index_language: "zh-CN"
contains_solutions: false
tags: ["最优控制", "动态规划", "Bellman", "HJB", "第三章作业"]
---

# 第三章作业题目

来源：[Homework for Chap3-2026.pdf](./Homework%20for%20Chap3-2026.pdf)，PDF 第 1 页，共 6 题。

整理方式：保留英文原题及题号，将公式转写为 LaTeX；中文标题、关键词和检索摘要为整理时补充。本文仅收录题目，不含解答。公式需在支持 LaTeX 数学公式的 Markdown 阅读器中查看。

## 题号目录与检索索引

| 题号 | 中文主题 | 检索关键词 | 题目要求摘要 |
| --- | --- | --- | --- |
| [3-1](#q3-1) | 一阶离散系统的动态规划 | discrete system；dynamic programming；量化；线性插值 | 手算最优控制律，并求初始状态为 1 时的最优控制序列 |
| [3-2](#q3-2) | Bellman 递推方程的推导 | Bellman；recurrence equation；动态规划递推关系 | 给出详细推导 |
| [3-3](#q3-3) | Kirk 例 3.10-1 的最优控制与增益变化 | optimal control gains；Figure 3-9(a)；MATLAB；第 84 页 | 求指定时刻的最优控制与最小代价，解释增益突增，并提供代码、数据及图形 |
| [3-4](#q3-4) | Kirk 教材习题 3.8 | exercise 3.8；第 99 页；详细推导 | 解答教材习题 3.8，并给出详细推导 |
| [3-5](#q3-5) | Kirk 教材习题 3.10 | exercise 3.10；第 99 页；详细推导 | 解答教材习题 3.10，并给出详细推导 |
| [3-6](#q3-6) | 有控制约束的 HJB 最小化 | Hamilton-Jacobi-Bellman；admissible control；二阶系统；控制幅值约束 | 根据状态、时间及价值函数的状态偏导，求最优可容许控制的表达式 |

引用时可使用“第三章作业 3-1”或“`Homework for Chap3-2026.md` 中的 3-6”。每道题设有固定锚点 `q3-1` 至 `q3-6`，便于建立链接。

<a id="q3-1"></a>

## 3-1 一阶离散系统的动态规划

**关键词：** dynamic programming；first-order discrete system；quantization；linear interpolation；动态规划；量化；线性插值。

### 原题

A first-order discrete system is described by the following equation:

$$
x(k+1)=-0.5x(k)+u(k).
$$

The performance measure to be minimized is

$$
J=\sum_{k=0}^{2}|x(k)|,
$$

and the admissible states and controls are constrained by

$$
-2\leq x(k)\leq 2,\qquad k=0,1,2,
$$

$$
-1\leq u(k)\leq 1,\qquad k=0,1.
$$

**(a)** Carry out by hand the computational steps required to determine the optimal control law by using dynamic programming. Quantize $u(k)$ and $x(k)$ in steps 0.5 and 1, respectively, about zero, and use linear interpolation.

**(b)** What is the optimal control sequence for an initial state value of 1?

<a id="q3-2"></a>

## 3-2 Bellman 递推方程的推导

**关键词：** dynamic programming；Bellman's recurrence equation；动态规划；Bellman 递推方程。

### 原题

Give the detailed derivation of the recurrence relation of the dynamic programming, i.e. the Bellman's recurrence equation.

<a id="q3-3"></a>

## 3-3 Kirk 例 3.10-1 的最优控制与增益变化

**关键词：** Kirk；Example 3.10-1；Figure 3-9(a)；optimal control sequence；minimum cost；control gains；MATLAB；最优控制增益。

**原题引用位置：** Kirk 参考书第 84 页，例 3.10-1，图 3-9(a)。

### 原题

It was mentioned on 84 of the reference book written by Kirk that "$F(N-k)$ goes to infinity as $k$ goes to infinity". However, in example 3.10-1 on Page 84, the plots for the optimal control gains are subject to sudden increases, see figure 3-9(a).

**(1)** Give the optimal control sequence for $k=0,1,2,3,$ and $4$ with the minimum cost values.

**(2)** Please explain why the control gains suddenly increases after $t=150\,\mathrm{s}$ in the figure, with Matlab codes, the resulted data and figure plots for $k\geq 300$.

**转写说明：** 上述关于 $F(N-k)$ 与 $k$ 的表述，以及 $t=150\,\mathrm{s}$、$k\geq 300$，均按作业 PDF 原文保留，未作解释性修正。

<a id="q3-4"></a>

## 3-4 Kirk 教材习题 3.8

**关键词：** Kirk；exercise 3.8；page 99；detailed derivation；教材习题 3.8。

**原题引用位置：** 参考书第 99 页，习题 3.8。

### 原题

Please give the solution to excises 3.8 on page 99, with a detailed derivation procedure.

<a id="q3-5"></a>

## 3-5 Kirk 教材习题 3.10

**关键词：** Kirk；exercise 3.10；page 99；detailed derivation；教材习题 3.10。

**原题引用位置：** 参考书第 99 页，习题 3.10。

### 原题

Please give the solution to excise 3.10 on page 99, with a detailed derivation procedure.

<a id="q3-6"></a>

## 3-6 有控制约束的 HJB 最小化

**关键词：** Hamilton-Jacobi-Bellman equation；HJB；admissible control；optimal control；state derivative of value function；控制幅值约束。

### 原题

The first step in using the Hamilton-Jacobi-Bellman equation

$$
0=J_t^*(x(t),t)
+\min_{u(t)}\left\{
g(x(t),u(t),t)
+J_x^*(x(t),t)\left[a(x(t),u(t),t)\right]
\right\}
$$

is to determine the admissible control $u^*(t)$ [in terms of $x(t)$, $t$ and $J_x^*$] that minimizes $\{\cdot\}$. Find $u^*(t)$ - expressed as a function of $x(t)$, $t$ and $J_x^*$, for the system

$$
\dot{x}_1(t)=0.5x_2(t),
$$

$$
\dot{x}_2(t)=-0.5x_1(t)+x_2(t)+u(t),
$$

and the performance measure

$$
J=\int_0^T\frac{1}{2}\left[q_1x_1^2+q_2x_2^2+u^2(t)\right]\,dt,
\qquad q_1,q_2>0.
$$

The admissible controls are constrained by

$$
|u(t)|\leq 1.
$$

## 教材引用索引

作业 PDF 中的第 84 页和第 99 页为所引用教材的页码，不是这份单页作业 PDF 的页码。作业 PDF 未附上例 3.10-1、习题 3.8 和习题 3.10 的完整内容，调用这些题目时需同时查阅教材。

本目录中的教材文件：[Donald E. Kirk - Optimal Control Theory: An Introduction，Chapter 3](./Donald%20E.%20Kirk-Optimal%20control%20theory_%20an%20introduction_Chapter%203%20%281%29.pdf)。

| 作业题号 | 教材定位信息 |
| --- | --- |
| 3-3 | 第 84 页；例 3.10-1；图 3-9(a) |
| 3-4 | 第 99 页；习题 3.8 |
| 3-5 | 第 99 页；习题 3.10 |
