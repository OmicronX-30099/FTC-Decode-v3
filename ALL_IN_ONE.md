<!-- BEGIN SOURCE: README.md -->

# ken-visiting-2026

Ken 的暑期研究项目：围绕组里正在冲刺投稿的论文（倾转旋翼安全过渡控制），承担仿真实验层与后续台架数据管线；上手文档、工作材料与全部交付物都在这个仓库。

## Reading order

1. `docs/onboarding/README.md` — 上手指南（环境、规矩、你在帮什么）
2. `docs/onboarding/01-nf-toy.md` — NF toy 热身教程（论文核心思想的最小可玩版）
3. `docs/onboarding/02-experiment-layer.md` — 论文仿真实验层交接（你的主战场）

## Layout

    docs/onboarding/     上手三件套
    deliverables/        你的代码与成果（nf-toy / experiment-layer / reports）
    FAILURES.md          失败记录（交付物）
    TASKS.md             当前任务（周五验收后更新）
    CHANGELOG.md         活动日志
    docs/DECISIONS.md    决策日志

交付去向：实验层产物（CSV/统计/图）经 Ray review 后合入论文仓库。


<!-- END SOURCE: README.md -->

<!-- BEGIN SOURCE: docs/onboarding/README.md -->

# 上手指南（第一天读这个）

> 你的路径分三个阶段：**阶段 0 环境 → 阶段 1 NF toy（懂思想）→ 阶段 2 论文实验层（干真活）**。
> 按自己的节奏推进——验收看的是 checkpoint 过没过，不看用了几天。
> 三份文档按顺序读：本文 → [01-nf-toy.md](01-nf-toy.md) → [02-experiment-layer.md](02-experiment-layer.md)。

## 你在帮什么（先看这段）

组里正在冲刺投稿一篇论文：**倾转旋翼飞行器从悬停转平飞时，某些倾转角附近控制能力会突然退化（一个矩阵掉秩），论文证明了一套带安全保证的方法能绕开这个危险区**。下文提到「论文」都指它（组里日常叫它 J2，听到这个词就是指这篇）。

你的三个阶段对应：

- **阶段 1**：在一个 2D 玩具问题上亲手实现论文方法的核心思想（Navigation Function）——玩懂它；
- **阶段 2**：给论文的仿真证据"补火力"——几百组随机初值的批量实验、统计和图（见 [02 号文档](02-experiment-layer.md)）；
- **之后**：仿真确认没问题，团队转台架实验，你负责数据侧管线。

## 阶段 0：环境（和 Ray 一起装）

1. **Python**：3.11+，只需要 `numpy` + `matplotlib`（建议用 `venv` 或 `uv` 建独立环境）。
2. **MATLAB**：装好并跑通一个 `disp('hello')`——论文的仿真核是 MATLAB，你至少要会"跑"它（后处理可以用 Python）。



<!-- END SOURCE: docs/onboarding/README.md -->

<!-- BEGIN SOURCE: docs/onboarding/01-nf-toy.md -->

# NF Toy 热身教程：亲手做一个"安全地形"

> 这是论文核心思想（Navigation Function）的最小可玩版。做完你能解释：什么是 NF、它为什么能带安全保证地绕开危险区、它在什么条件下会失效——然后你跑论文仿真时就知道自己在跑什么。
> 分四个阶段，按自己的节奏推进；每个阶段末尾有 checkpoint，自己核对，过了就进下一阶段。

## 阶段 0：先跑起来

不用从空白文件开始。仓库里有一个能直接跑的起点：

```bash
cd deliverables/nf-toy
python nf_toy_starter.py
```

你应该看到终端打印 `saved world.png`，打开它：一个圆盘、一个红色椭圆障碍、蓝色起点、绿色目标。**这就是你的世界**——圆盘代表可行的构型空间，红椭圆代表论文里那种"控制能力退化"的危险区（这里抽象成纯几何）。

打开 `nf_toy_starter.py` 读一遍：几何函数（`beta0/beta1/gamma`）和画图代码已经写好，你的全部工作就是按顺序填 4 个 TODO：

| TODO | 内容 | 阶段 |
|---|---|---|
| TODO-1 | `phi(q, K)` — Navigation Function 本体 | 一 |
| TODO-2 | `grad_phi(q, K)` — 中心差分梯度 | 二 |
| TODO-3 | `simulate(q0, K)` — 归一化梯度流仿真 | 二 |
| TODO-4 | `batch(n, K)` — 批量统计 | 三 |

每填对一个，重跑脚本就会多出新的图/输出。填 TODO 需要的公式全在下面。

## 问题设定（TODO-1 需要的全部数学）

二维配置空间，状态 $q=[x,y]$。物理上可以把 $x$ 想成归一化的倾转进度（hover→forward），$y$ 是一个辅助自由度——但做的时候完全可以当纯几何问题。

- **工作空间**（单位圆盘）：$\beta_0(q) = 1 - \lVert q \rVert^2$，$\beta_0>0$ 表示在内部。
- **障碍**（危险区椭圆）：$\beta_1(q) = (q-c)^\top A (q-c) - 1$，$c=[0,0]$，$A=\mathrm{diag}(1/a^2, 1/b^2)$，$a=0.30$，$b=0.45$。$\beta_1>0$ 表示在障碍外。
- **起点 / 目标**：$q_{\text{start}}=[-0.8, 0.35]$，$q_{\text{goal}}=[+0.8, 0]$。直线路径被障碍挡住，必须绕行。
- **Navigation Function（Rimon–Koditschek 形式）**：

$$\gamma(q)=\lVert q-q_{\text{goal}}\rVert^2,\quad \beta(q)=\beta_0(q)\,\beta_1(q),\quad \varphi(q)=\frac{\gamma(q)}{\bigl(\gamma(q)^K+\beta(q)\bigr)^{1/K}}$$

$\varphi\in[0,1]$：目标处 0，自由空间边界处 1。

- **控制律**（归一化梯度下降 + 显式 Euler）：

$$\dot q = -k\,\frac{\nabla\varphi(q)}{\lVert\nabla\varphi(q)\rVert + \epsilon}$$

常数都在脚本顶部（$k=0.05$，$dt=0.01$，$\epsilon=10^{-9}$，最大 5000 步）。

## 阶段一：把地形立起来（TODO-1）

1. 实现 `phi(q, K)`（$\beta_0,\beta_1,\gamma$ 已给，$\beta=\beta_0\beta_1$；先用默认 $K=4$）。
2. 重跑脚本——填对了就会多出 `phi_landscape.png`：左边等高线、右边 3D 地形。

**✅ Checkpoint 1**：等高线图上，目标处应是唯一"深谷"，障碍和外边界应是"高墙"。画图代码已经把自由空间外 mask 掉了——**去读 `plot_phi_landscape` 里那个 `in_free_space` 判断，想明白它在防什么**（答案是陷阱 1）。

**⚠️ 陷阱 1（静默 NaN）**：障碍内 $\beta<0$，$(\gamma^K+\beta)^{1/K}$ 对负数开分数次幂时 numpy **静默返回 NaN 不报错**。画图靠 mask 防；仿真时要每步先检查下一状态是否仍在自由空间，越界就终止该轨迹并记一次 violation。

## 阶段二：让点动起来（TODO-2 + TODO-3）

1. 实现 `grad_phi`：**中心差分**（$h=10^{-6}$），不要先推解析式——把"仿真循环对不对"和"链式法则对不对"这两个问题解耦。
2. 实现 `simulate`：控制律 + Euler 循环。从 $q_{\text{start}}$ 跑一条轨迹，在 `main()` 里把它画到等高线图上（这一步自己写画图，照着 `plot_world` 依样画葫芦）。
3. 判据（脚本顶部常数已给）："到达" $\lVert q-q_{\text{goal}}\rVert<0.05$；"卡住" $\lVert\nabla\varphi\rVert<10^{-8}$（**不要**用速度小判卡住——归一化梯度下速度恒定）；每步先检查下一状态是否仍在自由空间。
4. 扫 $K\in\{2,4,8,16\}$，画四条轨迹对比。

**✅ Checkpoint 2**：轨迹应绕过障碍到达目标；不同 $K$ 下绕行的"保守程度"不同。

**⚠️ 陷阱 2（K 不单调）**："$K$ 越大越好"只在一定范围内成立——$K$ 太大时浮点会灾难性抵消（远处梯度可能精确下溢为 0）。$K=16$ 不一定优于 $K=8$，非单调**不代表你有 bug**。

**⚠️ 陷阱 3（对称初值）**：如果起点与目标、障碍中心共线（比如都在 $y=0$ 轴上），由对称性轨迹会永远卡在轴上——任何方法任何 $K$ 都一样。这是几何巧合不是 bug。starter 的起点特意放在离轴的 $[-0.8,0.35]$。

## 阶段三：从"一条轨迹"到"统计说话"（TODO-4）

1. 实现 `batch`：在圆盘内均匀撒 **50 个随机起点**（**注意**：圆盘均匀采样要用 $r=\sqrt{u},\ u\sim U(0,1)$ 配均匀角度——$r$ 直接均匀会把点堆向圆心。想清楚为什么：这个"均匀 ≠ 逐坐标均匀"的思想，你在论文实验层每天都要用）。筛掉落在障碍内的点。
2. 固定种子跑批量，统计：成功率、violation 数、平均步数、平均路径长度。$K\in\{2,4,8,16\}$ 各跑一遍，出一张对比表。
3. 找出**至少一个失败案例**（NaN / 卡住 / violation），画出来，写一段归因进 `FAILURES.md`。

**✅ Checkpoint 3（验收件）**：一键重跑的脚本 + 对比表 + 轨迹束图 + 失败案例图 + 半页总结："这个势场为什么能保证安全？$K$ 扮演什么角色？什么情况下它也救不了？"

## 做完后你应该能回答的问题（验收时会问）

1. $\varphi$ 为什么设计成 $\gamma/(\gamma^K+\beta)^{1/K}$ 这个形状？分子分母各管什么？
2. NF 声称"无伪极小"——你的实验里它成立吗？什么条件下会破坏？
3. 为什么归一化梯度？原始梯度下降会怎样？
4. 把这个 2D toy 的"椭圆障碍"换成论文里真实的"控制权限退化区"，还缺什么？（没有标准答案——带着它去读 [02 号文档](02-experiment-layer.md)。）


<!-- END SOURCE: docs/onboarding/01-nf-toy.md -->

<!-- BEGIN SOURCE: docs/onboarding/02-experiment-layer.md -->

# 论文仿真实验层：你的主战场

> 前置：做完 [NF toy 热身教程](01-nf-toy.md)。toy 里的 $\varphi$ 就是论文里的核心对象——只是配置空间从 2D 圆盘换成了姿态×倾转角流形 $\mathrm{SO}(3)\times\mathcal C$，障碍从椭圆换成了真实的"控制分配奇异区"。

## 一、论文在证什么（速览版）

倾转旋翼机的三个旋翼可以倾转。给定倾转角 $\theta=(\theta_1,\theta_2,\theta_3)$，推力到机体力矩的映射是一个 3×3 矩阵 $B_\tau(\theta)$。在对称倾转路径上 $\theta\approx44.4°$ 附近，$\det B_\tau\to0$——**矩阵掉秩，飞机暂时失去产生某些力矩的能力**。这就是危险区。

论文在 $\mathrm{SO}(3)\times\mathcal C$ 上构造一个 Navigation Function $\phi$（把危险区编码成障碍），再通过能量整形（port-Hamiltonian）把 $\phi$ 变成闭环的势能，证明：能量不超过门槛 $c<\kappa$ 的初始状态，**永远不会碰到危险区，且几乎全部收敛到目标构型**。这个"能量漏斗"记作 $\mathcal D_c$。

你不需要看懂证明。你需要记住三个对象：**安全集 $\mathcal F$**（$\tilde\sigma\ge\varepsilon$ 的构型区）、**漏斗 $\mathcal D_c$**（能量 $H_d\le c$ 的目标连通分量）、**目标 $z^\star$**。你的实验就是在数值上展示：漏斗里的轨迹不出安全集、能量单调降、收敛到目标。

## 二、分层与接口（记住这个约定）

| 层 | 谁 | 内容 |
|---|---|---|
| 物理核 | Ray | 动力学、积分器、$\phi$ / $H_d$ 计算——**你不改这层**；发现疑似 bug 直接报 Ray，不自己修 |
| 实验层 | **你（完整拥有）** | 初值采样、批量运行、数据管理、统计、图、复现 README |

接口就一个函数 + 一个配置：

```
[t, Z, Hd, margins] = run_one(z0, config)   % Ray 提供,MATLAB
```

输入初始状态和配置，返回轨迹、能量历史、安全裕度历史。你的一切工作都建立在这个接口之上。
**交付方式**：Ray 会把物理核（`run_one` + 依赖文件）放进本仓库的 `lib/` 目录；在它到位之前，**先做 D1 采样器——它不依赖这个接口**，正好是你的起手式。**后处理语言自选**：可以全 MATLAB，也可以 MATLAB 跑批量、导 CSV、用 Python(pandas/matplotlib) 做统计和图——接口是 CSV,两边解耦。

## 三、你的四个交付物

### D1｜初值采样器

- **倾转部分**：$\theta$ 在盒内均匀采样（toy 阶段三的 $r=\sqrt u$ 教训在这里的对应物：均匀不等于每个坐标各自均匀,想清楚测度）。
- **姿态部分（核心技能点）**：$\mathrm{SO}(3)$ 上均匀采样——**用随机单位四元数**（4 个独立标准高斯归一化），**不要**用欧拉角各自均匀（会向极点堆积,这是经典错误）。
- **筛选**：采出的全状态先过两道筛——在 $\mathcal F$ 内、$H_d(z_0)\le c$（$c=0.92\kappa$，config 里给）。记录并报告筛选通过率。
- 固定种子;两次运行产生完全相同的初值集。

### D2｜批量运行器 + 数据规范

- N≥200 组初值 × {NF 律, log-barrier 基线} 逐条跑,结果落 CSV。
- CSV 每行一条轨迹,至少含：`ic_id, seed, law, converged, t_conv, min_sigma_margin, min_det, Hd_monotone, violation, exit_reason`。
- 轨迹级时间序列另存（抽样存,别存爆磁盘）;任何一行都能由 `ic_id+seed+config` 精确复跑。

### D3｜统计表

- 每种控制律一列:收敛率 / $\min_t\tilde\sigma$ 的中位数与最小值 / 违约计数（**应为 0**,不为 0 立刻报 Ray——那可能是重要发现）/ 收敛时间分布 / $H_d$ 非单调计数（数值容差内）。
- 换一个种子整体重跑,统计量应在声明容差内一致——这一条本身写进表格脚注。

### D4｜图（出版级）

- 批量包络图：姿态误差 / 倾转角 / 最小裕度随时间的分布带（中位数 + 分位数带,不是 200 条线糊在一起）。
- 统一样式：矢量输出（PDF）、字号与论文正文一致、每张图有生成脚本一键再生。
- **每张图先答一句"它支持论文里哪个 claim"**——答不上的图不画。这是组里的铁律。

## 四、验收标准（周五 gate 用）

1. 干净 clone 你的仓库 → README 一条命令 → D1–D4 全部再生成,结果与你提交的一致。
2. 200 组初值:0 违约、$H_d$ 单调（容差内）、收敛率与"测度零"预期一致（未收敛的逐条归因）。
3. 抽查任一 `ic_id`:能单条复跑并和 CSV 里的行对上。
4. 当面解释:SO(3) 均匀采样为什么用四元数;筛选通过率意味着什么;任一张图支持哪个 claim。

## 五、边界与升级

- 发现物理核疑似 bug（NaN、正交性漂移 $\|R^\top R-I\|>10^{-9}$、能量莫名上升）:**停,记录现象,报 Ray**。不猜不修。
- 你的结果推翻了论文现文的某个说法（比如基线对比方向反了）:**这是好事,立刻报**,先报后改。
- 实验层内的一切（采样、数据格式、画图方案）:你做主,做完周五一起 review。


<!-- END SOURCE: docs/onboarding/02-experiment-layer.md -->

<!-- BEGIN SOURCE: TASKS.md -->

# TASKS — ken-visiting-2026

> Current Focus 每周五验收后更新。

## Current Focus

**Onboarding**：阶段 0 环境（Python/MATLAB/git，与 Ray 一起）→ NF toy 热身（`docs/onboarding/01-nf-toy.md`，Checkpoint 3 = 首个周五验收件）→ 读 `02-experiment-layer.md` 接手论文仿真实验层。

## Next

1. 实验层 D1：初值采样器（SO(3) 四元数均匀采样 + 倾转盒 + 双重筛选）——不依赖物理核，随时可开工。
2. D2–D4：批量运行器 / 统计表 / 出版级图（验收标准见 onboarding 02 §四）。
3. 下一阶段（仿真验收过后）：台架实验数据管线。

## Blocked / Waiting

- `run_one(z0, config)` 物理核：由 Ray 抽出后放入本仓库 `lib/`（在此之前先做 D1）。

## Done

- 项目建立；onboarding 三件套 + toy 脚手架就位（2026-07-18）。


<!-- END SOURCE: TASKS.md -->

<!-- BEGIN SOURCE: deliverables/README.md -->

# deliverables — 你的代码和成果都放这里

```
nf-toy/            NF toy 热身：代码（starter 已就位）、图、总结（Checkpoint 3 验收件）
experiment-layer/  论文实验层：D1 采样器 / D2 批量+CSV / D3 统计 / D4 图
reports/           阶段总结、失败分析、最终报告
```

- 每个子目录自带一个 README：怎么跑、依赖什么、种子是多少。
- 失败记录写仓库根的 `FAILURES.md`。


<!-- END SOURCE: deliverables/README.md -->

<!-- BEGIN SOURCE: FAILURES.md -->

# FAILURES — 失败记录（交付物，不是耻辱柱）

> 格式：日期 | 现象 | 猜测原因 | 试过的修复 | 状态（open/解决）。新条目置顶。

<!-- 示例：
## 2026-07-21 | toy Day1 势场图障碍内出现 NaN 花纹
- 现象：φ 等高线在椭圆内部全是空洞
- 猜测：β<0 时分数次幂静默 NaN（教程陷阱 1）
- 修复：画图前 mask 自由空间外区域 → 解决
-->


<!-- END SOURCE: FAILURES.md -->

<!-- BEGIN SOURCE: docs/DECISIONS.md -->

# DECISIONS — ken-visiting-2026

> Append-only，新条目置顶。Type: project / scope / code。

## 2026-07-18 | D-001 | project | 建立项目与实验层接口约定

Type: project

本项目承载 Ken 的全部工作与交付，当前完全面向组里冲刺投稿的论文。接口约定（见 `onboarding/02-experiment-layer.md`）：实验层（初值采样 / 批量 / 统计 / 图）由 Ken 完整负责；物理核（动力学 / 积分器 / 能量与势函数计算）由 Ray 维护并以 `lib/` 交付，Ken 不修改物理核，疑似问题一律上报。


<!-- END SOURCE: docs/DECISIONS.md -->
