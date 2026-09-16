# DCPTilt v3.29+ 当前实验版参数说明

> 对应代码状态：`v3.29_AP_YAW_RATE` 基础上加入 **TD3 Shadow Inference**、**TD3 30/TIME 时间放缩**，并采用当前 6 s 转场实验调参。  
> 参数来源：当前 `ArduPlane/tiltrotor.cpp` 中新增/修改的 `DCPT_*` 参数定义。  
> 完整参数名均带 ArduPilot Tiltrotor 前缀，因此源码中的 `DCPT_TIME` 在地面站中显示为 `Q_TILT_DCPT_TIME`。
>
> **注意：** 下表“范围”是代码中 `@Range` 声明的可设置范围，不等于“实机安全范围”。实机应先在 SITL、无桨台架和低风险飞行中逐步验证。

---

## 0. 本版本相对 v3.29 的主要变化

当前实验版在原 v3.29 基础上增加/确认了以下内容：

1. 新增 TD3 Shadow 推理：
   - `Q_TILT_DCPT_TDSH`
   - `Q_TILT_DCPT_TDSA`
2. 在线 TD3 和 Shadow TD3 都使用同一套时间放缩：
   \[
   k_t=\frac{30}{Q\_TILT\_DCPT\_TIME}
   \]
3. 当 `TIME=6 s` 时，TD3 倾转积分速率相对原 30 s 网络放大 5 倍。
4. Shadow TD3 仅在 `PROF=0~5` 时运行，不改变真实倾转角、控制器权重、油门、高度控制或转场结束条件。
5. `PROF=6~8` 为在线 TD3，因此不会再额外运行 Shadow，避免重复执行网络。
6. `Q_TILT_DCPT_EN=0` 的 ArduPilot 官方前向倾转目前 **不能运行 Shadow TD3**。
7. `AUTO` 自动航线中，只要任务实际触发一次 **VTOL→FW 前向转场**，并且 `Q_TILT_DCPT_EN=1`，就会进入自定义 DCPTilt；并不要求必须由 FBWA 触发。
8. **FW→VTOL 反向转场始终使用 ArduPilot 官方逻辑**。
9. 当前 6 s 实验为减小末端 Pitch 抬头，常用：
   ```text
   Q_TILT_DCPT_TWIN 0.25
   Q_TILT_DCPT_TGN  0.5
   Q_TILT_DCPT_FWAP 1.5
   Q_TILT_DCPT_FWAD 2.5
   ```

---

## 1. 总体开关、转场时间和实验选择

| 参数 | 默认值 | 范围/可选值 | 含义 | 当前建议 |
|---|---:|---|---|---|
| `Q_TILT_DCPT_EN` | `0` | `0/1` | 选择前向转场方式。`0`=ArduPilot 原生倾转转场；`1`=DCPTilt 自定义前向转场。选择在每次已解锁前向转场开始时锁存，中途修改不会切换当前转场。 | 自定义实验设 `1`；AP 对照组设 `0`。 |
| `Q_TILT_DCPT_TIME` | `30 s` | `1~120 s` | DCPT 前向转场总时长。0~5 号轨迹按该时间归一化；在线/Shadow TD3 也根据该时间进行 `30/TIME` 时间放缩。 | 当前短时实验使用 `6 s`。 |
| `Q_TILT_DCPT_HNDT` | `3 s` | `0~10 s` | DCPT 主转场结束后，倾转电机实际输出向固定翼油门需求平滑交接的时间。`0` 关闭该平滑。 | 一般保持 `3 s`。 |
| `Q_TILT_DCPT_MODE` | `0` | `0:FUZZ` `1:SWITCH` `2:NMPC` `3:FIS` | 选择 **MC/FW 控制器权重分配策略**。它与倾转轨迹 `PROF` 独立。 | 当前主要使用 `3:FIS`。 |
| `Q_TILT_DCPT_PROF` | `0` | `0:Linear` `1:Smoothstep` `2:POptA` `3:POptB` `4:POptC` `5:POptD` `6:TD3A` `7:TD3B` `8:TD3C` | 选择倾转轨迹。0~5 为预设时序轨迹，6~8 为 20 Hz TD3 Actor 在线增量积分轨迹。 | 当前重点比较 `0 / 5 / 8`。 |

### 1.1 MODE 与 PROF 的关系

两者功能不同：

- `MODE`：决定多旋翼控制器 `MCW` 和固定翼控制器 `FWW` 如何分权；
- `PROF`：决定电机/短舱倾转角如何变化。

例如：

```text
MODE = 3  (FIS)
PROF = 0  (Linear)
```

或：

```text
MODE = 3  (FIS)
PROF = 8  (TD3C)
```

都是合法组合。

---

## 2. AUTO 自动航线中的触发逻辑

DCPTilt 不是“FBWA 专用功能”，而是替换 **前向 VTOL→FW transition** 的控制路径。

### 2.1 AUTO 中什么时候使用自定义倾转

如果：

```text
Q_TILT_DCPT_EN = 1
```

并且 AUTO 任务实际发生：

```text
VTOL / QTAKEOFF 状态
        ↓
进入固定翼航段 / 普通固定翼航点
        ↓
触发 VTOL → FW transition
```

则这一段前向转场会走 DCPTilt，并使用当前：

```text
Q_TILT_DCPT_MODE
Q_TILT_DCPT_PROF
Q_TILT_DCPT_TIME
```

因此，例如：

```text
QTAKEOFF
↓
NAV_WAYPOINT
↓
NAV_WAYPOINT
↓
QLAND
```

典型流程是：

```text
QTAKEOFF
    ↓
多旋翼起飞
    ↓
VTOL → FW
    ↓
【DCPTilt 自定义前向倾转】
    ↓
固定翼 AUTO 航线
    ↓
FW → VTOL
    ↓
【ArduPilot 官方反向倾转】
    ↓
QLAND
```

### 2.2 不会触发 DCPTilt 的情况

- 进入 AUTO 时已经处于固定翼状态，没有发生新的 VTOL→FW 转场；
- 整条任务均为 VTOL/Q 类任务，没有进入固定翼航段；
- `Q_TILT_DCPT_EN=0`。

### 2.3 反向转场

当前代码明确规定：

\[
FW\rightarrow VTOL
\]

**始终交给 ArduPilot 官方 SLT/Tiltrotor 返回逻辑。**

这一点用于保留 Q 模式返回、Q_ASSIST 和降落阶段的官方安全逻辑。

---

## 3. 公共高度/纵向控制参数

这组参数决定转场期间定高，以及固定翼 Pitch 支路目标生成。

| 参数 | 默认值 | 范围 | 单位 | 含义 | 当前 6 s 实验值 |
|---|---:|---:|---|---|---:|
| `Q_TILT_DCPT_ALTP` | `0.50` | `0~3` | `(m/s²)/m` | 高度误差到垂向加速度修正的 P 增益。 | `0.50` |
| `Q_TILT_DCPT_ALTD` | `0.80` | `0~5` | — | 垂向速度阻尼增益。 | `0.80` |
| `Q_TILT_DCPT_AMAX` | `3.0` | `0.2~6` | `m/s²` | 高度环允许生成的最大绝对垂向加速度修正。 | `3.0` |
| `Q_TILT_DCPT_FWAP` | `2.0` | `0~10` | `deg/m` | 固定翼 Pitch 目标对高度误差的 P 增益。 | **`1.5`** |
| `Q_TILT_DCPT_FWAD` | `3.0` | `0~15` | `deg/(m/s)` | 固定翼 Pitch 目标对垂向速度的阻尼增益。 | **`2.5`** |
| `Q_TILT_DCPT_PMAX` | `15` | `3~30` | `deg` | DCPT 高度控制器允许生成的最大绝对固定翼 Pitch 目标。 | `15` |

固定翼纵向支路大致为：

\[
\theta_{FW,cmd}
=
K_h e_h-K_vV_z
\]

再由 ArduPlane 原生固定翼 Pitch Controller 跟踪，并乘固定翼控制权重 `FWW`。

### 3.1 当前 6 s Pitch 调参说明

原 6 s TD3C 实验末段存在约 3° 以上抬头。当前优先通过参数减弱终端高度修正和固定翼 Pitch 高度环：

```text
Q_TILT_DCPT_FWAP 1.5
Q_TILT_DCPT_FWAD 2.5
Q_TILT_DCPT_TWIN 0.25
Q_TILT_DCPT_TGN  0.5
```

这组参数用于当前实验，并不意味着应修改代码默认值。

---

## 4. 机翼升力估计、总推力和终端高度修正

| 参数 | 默认值 | 范围 | 单位 | 含义 | 当前 6 s 实验值 |
|---|---:|---:|---|---|---:|
| `Q_TILT_DCPT_VLFT` | `22` | `5~40` | `m/s` | 简化机翼升力模型参考速度。近似 `Lwing/mg=(V/VLFT)^2`，再由 `LMAX` 限幅。 | `22` |
| `Q_TILT_DCPT_CREG` | `0.12` | `0.03~0.5` | — | `sqrt(cos²(theta)+CREG²)` 正则项，避免大倾角推力补偿奇异。 | `0.12` |
| `Q_TILT_DCPT_LMAX` | `0.95` | `0.5~1.0` | — | 最大建模机翼升力占重力比例。 | `0.95` |
| `Q_TILT_DCPT_TFLT` | `0.15` | `0~1` | `s` | 公共总推力指令一阶滤波时间常数。 | `0.15` |
| `Q_TILT_DCPT_TWIN` | `0.25` | `0.05~0.50` | 转场比例 | 终端高度预测器在最后多少比例的转场中介入。 | **`0.25`** |
| `Q_TILT_DCPT_TGN` | `1.0` | `0~3` | — | 终端预测高度误差增益。`0` 关闭终端修正。 | **`0.5`** |

终端预测器：

\[
h_{pred}=h+V_z t_{remain}
\]

注意这里的 `t_remaining` 是**当前设定转场时间的剩余时间**。因此当 `TIME=6 s` 时，预测器针对 6 s 末端，而不是固定针对 30 s。

---

## 5. Yaw 参数 —— 当前结构

当前版本不再使用早期“航向误差 PD 直接打方向舵”的方案。

固定翼侧：

```text
共同 Yaw 目标
    ↓
航向误差
    ↓
DCPT_YHP 外环 P
    ↓
目标 Yaw Rate
    ↓
ArduPlane 原生 yawController Rate PID
    ↓
Rudder 原始输出
    ↓
× FWW
```

多旋翼侧：

```text
AP Multicopter Yaw Controller
    ↓
× MCW
```

### 5.1 当前有效 Yaw 参数

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_YHP` | `1.0` | `0~4` | `1/s` | 航向误差（deg）到固定翼目标 Yaw Rate（deg/s）的外环 P。 |
| `Q_TILT_DCPT_YRM` | `20` | `1~60` | `deg/s` | 固定翼 Yaw 支路最大目标偏航角速度。 |
| `Q_TILT_DCPT_YMAX` | `3000` | `0~4500` | AP scaled output | 固定翼方向舵原始输出限幅，之后再乘 `FWW`。 |

### 5.2 Legacy 参数

| 参数 | 默认值 | 范围 | 状态 |
|---|---:|---:|---|
| `Q_TILT_DCPT_YAWP` | `100` | `0~400` | Legacy，仅兼容旧参数；当前 FW Yaw 不使用。 |
| `Q_TILT_DCPT_YAWD` | `150` | `0~500` | Legacy，仅兼容旧参数；当前 FW Yaw 不使用。 |

---

## 6. Hard Switch 模式专用参数

仅在：

```text
Q_TILT_DCPT_MODE = 1
```

时主要生效。

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_SWLO` | `6` | `0~40` | `m/s` | 低速阈值。低于它：`MCW=1, FWW=0`。 |
| `Q_TILT_DCPT_SWHI` | `17` | `0.5~50` | `m/s` | 高速阈值。高于它：`MCW=0, FWW=1`。 |
| `Q_TILT_DCPT_SWMD` | `0.50` | `0~1` | — | SWLO~SWHI 中间区固定翼权重。 |
| `Q_TILT_DCPT_SWPK` | `4.0` | `0~10` | `deg` | FWW 正向阶跃时附加的固定翼 Pitch nose-down kick 增益。 |
| `Q_TILT_DCPT_SWKT` | `1.2` | `0.2~3` | `s` | Pitch kick 平滑衰减时间。 |

简化逻辑：

```text
V <= SWLO       : MCW=1,       FWW=0
SWLO<V<SWHI     : MCW=1-SWMD,  FWW=SWMD
V >= SWHI       : MCW=0,       FWW=1
```

---

## 7. “Fake NMPC” 模式专用参数

仅在：

```text
Q_TILT_DCPT_MODE = 2
```

时使用。

MODE 2 的控制权重本身复用 MODE 0 的 FUZZ 分配，额外加入早期 Pitch 瞬态。

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_NPIT` | `35` | `0~80` | `deg` | 早期 `MCW*sin(tilt)` Pitch bias 增益。 |
| `Q_TILT_DCPT_NMAX` | `4` | `0~20` | `deg` | 额外 Pitch bias 最大值。 |

该额外瞬态在约 35% 转场进度前平滑衰减到 0。

---

## 8. TD3 Actor 参数

在线 TD3 对：

```text
PROF = 6 / 7 / 8
```

生效。

Actor 网络运行频率固定：

\[
20\ Hz
\]

即每：

\[
50\ ms
\]

执行一次网络前向推理。

### 8.1 TD3S 与 30/TIME 时间放缩

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_TD3S` | `0.084` | `0~1` | `1/s` | TD3 基础倾转速率尺度，定义在原始 30 s 参考转场上。 |

当前不再直接使用：

\[
\dot{\lambda}=TD3S\cdot a_k
\]

而是：

\[
k_t=\frac{30}{T}
\]

\[
\dot{\lambda}
=
TD3S\cdot k_t\cdot a_k
\]

其中：

\[
T=Q\_TILT\_DCPT\_TIME
\]

离散积分：

\[
\lambda_{k+1}
=
\lambda_k+
TD3S\cdot
\frac{30}{T}
\cdot a_k\cdot0.05
\]

### 8.2 常用时间对应关系

| `Q_TILT_DCPT_TIME` | 时间放缩 | `TD3S=0.084` 等效速率系数 |
|---:|---:|---:|
| `30 s` | `1×` | `0.084` |
| `10 s` | `3×` | `0.252` |
| `6 s` | **`5×`** | **`0.420`** |
| `5 s` | `6×` | `0.504` |
| `3 s` | `10×` | `0.840` |

**重要：**

- Actor 网络权重没有变化；
- Actor 推理仍然是 20 Hz；
- 放缩的是 Actor 输出到 `lambda_dot` 的积分速率；
- `TIME=30 s` 时完全恢复原始速率；
- `TIME=6 s` 时在线 TD3 和 Shadow TD3 都使用 5× 积分速率。

### 8.3 转场时间结束

`Q_TILT_DCPT_TIME` 到达后，主转场状态机结束。

如果网络自然积分的：

\[
\lambda <1
\]

完成逻辑仍会将最终倾转目标补到完全前向位置。

因此时间放缩的目标是让网络在设定时间附近自然接近 1，减少末端强制补齐。

---

## 9. TD3 速度与高度观测参数

### 9.1 速度观测

| 参数 | 默认值 | 范围/可选值 | 单位 | 含义 | 当前实验值 |
|---|---:|---|---|---|---:|
| `Q_TILT_DCPT_TD3V` | `1` | `0:LegacyStrategy` `1:AirspeedAuto` `2:Groundspeed` `3:NED3D` | — | Actor 速度输入来源。 | `1` |
| `Q_TILT_DCPT_TD3F` | `25` | `5~50` | `m/s` | 当前飞机典型平飞速度，用于平台速度尺度映射。 | **`18`** |
| `Q_TILT_DCPT_VEXP` | `1.0` | `1~2` | — | TD3 速度归一化指数。 | **`2`** |

当前速度归一化：

\[
V_n=
1.5
\left(
\frac{V_{used}}{V_{flat}}
\right)^{VEXP}
\]

其中：

- `Vused` 由 `TD3V` 决定；
- `Vflat = TD3F`。

### 9.2 高度误差观测

| 参数 | 默认值 | 范围 | 单位 | 含义 | 当前实验值 |
|---|---:|---:|---|---|---:|
| `Q_TILT_DCPT_EHF` | `0` | `0~60` | `s` | 到指定时间冻结 Actor 使用的 Eh。`0` 关闭。 | `0` |
| `Q_TILT_DCPT_EHL` | `0.5` | `0~2` | `m` | 只限制送给 Actor 的 Eh 绝对值；`0` 不裁剪。 | **`0`** |

Eh 定义：

\[
Eh=h_{ref}-h_{actual}
\]

所以：

- `Eh>0`：飞机低于参考高度；
- `Eh<0`：飞机高于参考高度。

---

## 10. Shadow TD3

### 10.1 开关和 Actor 选择

| 参数 | 默认值 | 范围/可选值 | 含义 |
|---|---:|---|---|
| `Q_TILT_DCPT_TDSH` | `0` | `0:Disabled` `1:Enabled` | 开启/关闭后台 Shadow TD3。 |
| `Q_TILT_DCPT_TDSA` | `8` | `6:TD3A` `7:TD3B` `8:TD3C` | 选择后台执行的 Actor。当前主要使用 TD3C。 |

### 10.2 Shadow 可以在哪些情况下运行

当前实现：

| 实际转场 | Shadow TD3 |
|---|---|
| AP 官方倾转，`DCPT_EN=0` | **不能运行** |
| `PROF=0~5` | **可以运行** |
| `PROF=6~8` 在线 TD3 | 不重复运行 Shadow |

即：

```text
DCPT_EN=1
PROF=0~5
TDSH=1
```

才是当前标准 Shadow 使用方式。

即使在 AP 官方组设置：

```text
Q_TILT_DCPT_TDSH = 1
```

当前代码也不会真正执行 Shadow，因为官方前向转场不会进入 DCPT 静态轨迹分支。

### 10.3 Shadow 不会影响什么

Shadow Actor 的输出不会写入：

- 实际 `target_tilt`
- 实际 `current_tilt`
- 舵机输出
- `MCW/FWW`
- 高度控制器
- 转场完成状态
- 实际油门/控制器分配

所以 Shadow 的目的只有：

1. 评估飞控端神经网络实时计算负载；
2. 记录如果 TD3 在当前飞行状态下在线运行，会输出什么；
3. 与真实 `PROF=0~5` 倾转轨迹进行离线比较。

### 10.4 Shadow 时间放缩

Shadow 的私有 `lambda` 与在线 TD3 使用相同公式：

\[
\dot{\lambda}_{shadow}
=
TD3S
\cdot
\frac{30}{TIME}
\cdot
ActorOutput
\]

因此：

```text
TIME=6 s
```

时，Shadow TD3 同样模拟 5× 时间压缩后的 TD3 运行状态。

注意：这只影响 Shadow 内部记录的私有 `lambda`，不会改变真实飞机倾转。

---

## 11. TD3 / Shadow 日志

### 11.1 在线 TD3：`RLT`

在线 `PROF=6~8` 记录：

```text
RLT:
TimeUS
ActorUS
ProjUS
TotalUS
Miss
Seq
```

重点指标：

- `ActorUS`：神经网络前向推理耗时；
- `ProjUS`：Actor 输出到倾转增量映射/积分耗时；
- `TotalUS`：本次在线 TD3 计算总耗时；
- `Miss`：是否超过 20 Hz 的 50 ms 预算；
- `Seq`：样本序号。

### 11.2 Shadow TD3：`RLSH`

```text
RLSH:
TimeUS
ActorUS
TotalUS
PeriodUS
JitUS
Miss
Seq
```

推荐统计：

```text
ActorUS: mean / median / P95 / P99 / max
TotalUS: mean / median / P95 / P99 / max
PeriodUS
JitUS
Miss
```

20 Hz 实时预算为：

\[
50\,000\ \mu s
\]

理想情况：

```text
Miss = 0
PeriodUS ≈ 50000
```

### 11.3 Shadow 行为：`DTSH`

```text
DTSH:
TimeUS
Act
Prof
Prog
TiltT
Eh
VUse
Vn
Mn
Out
Lam
```

其中：

- `Act`：Shadow Actor 编号；
- `Prof`：真实控制飞机的 0~5 号轨迹；
- `TiltT`：真实轨迹目标；
- `Out`：TD3 Actor 输出；
- `Lam`：经过当前 `30/TIME` 放缩后的 Shadow 私有倾转状态。

---

## 12. 当前推荐 6 s TD3C 实验公共参数

当前用于短时转场实验的一组公共设置：

```text
Q_TILT_DCPT_EN    1
Q_TILT_DCPT_MODE  3
Q_TILT_DCPT_TIME  6

Q_TILT_DCPT_HNDT  3

Q_TILT_DCPT_ALTP  0.5
Q_TILT_DCPT_ALTD  0.8
Q_TILT_DCPT_AMAX  3.0

Q_TILT_DCPT_FWAP  1.5
Q_TILT_DCPT_FWAD  2.5
Q_TILT_DCPT_PMAX  15

Q_TILT_DCPT_VLFT  22
Q_TILT_DCPT_CREG  0.12
Q_TILT_DCPT_LMAX  0.95
Q_TILT_DCPT_TFLT  0.15

Q_TILT_DCPT_TWIN  0.25
Q_TILT_DCPT_TGN   0.5

Q_TILT_DCPT_YHP   1.0
Q_TILT_DCPT_YRM   20
Q_TILT_DCPT_YMAX  3000

Q_TILT_DCPT_TD3V  1
Q_TILT_DCPT_TD3F  18
Q_TILT_DCPT_TD3S  0.084
Q_TILT_DCPT_VEXP  2

Q_TILT_DCPT_EHF   0
Q_TILT_DCPT_EHL   0
```

其中 `TD3S=0.084` 不需要手工改成 `0.420`。

因为：

```text
TIME=6
```

时，代码内部自动计算：

\[
0.084\times\frac{30}{6}
=
0.420
\]

---

## 13. 当前四组主要对比实验

### 13.1 AP 官方倾转

```text
Q_TILT_DCPT_EN   0
Q_TILT_DCPT_TDSH 0
```

说明：

- 使用 ArduPilot 官方前向倾转；
- 当前代码不能同时运行 Shadow TD3；
- 作为原生基线组。

### 13.2 PROF0 Linear + MODE3 + 6 s

```text
Q_TILT_DCPT_EN    1
Q_TILT_DCPT_MODE  3
Q_TILT_DCPT_PROF  0
Q_TILT_DCPT_TIME  6

Q_TILT_DCPT_TDSH  1
Q_TILT_DCPT_TDSA  8
```

实际飞机：

```text
Linear 6 s
```

后台：

```text
TD3C Shadow
+
30/6 = 5× 时间放缩
```

### 13.3 PROF5 POptD + MODE3 + 6 s

```text
Q_TILT_DCPT_EN    1
Q_TILT_DCPT_MODE  3
Q_TILT_DCPT_PROF  5
Q_TILT_DCPT_TIME  6

Q_TILT_DCPT_TDSH  1
Q_TILT_DCPT_TDSA  8
```

实际飞机：

```text
POptD 6 s
```

后台：

```text
TD3C Shadow
+
5× 时间放缩
```

### 13.4 PROF8 TD3C Online + MODE3 + 6 s

```text
Q_TILT_DCPT_EN    1
Q_TILT_DCPT_MODE  3
Q_TILT_DCPT_PROF  8
Q_TILT_DCPT_TIME  6

Q_TILT_DCPT_TDSH  0
```

实际飞机由 TD3C 在线控制倾转：

```text
TD3C Actor
↓
20 Hz
↓
TD3S × 30/6
↓
lambda integration
↓
真实倾转
```

因为本身已经执行在线 Actor，不再额外打开 Shadow。

---

## 14. 0~5 号轨迹说明

### PROF0 — Linear

\[
\lambda=p
\]

其中：

\[
p=\frac{t}{Q\_TILT\_DCPT\_TIME}
\]

所以在 6 s 实验中为匀速：

\[
0^\circ\rightarrow90^\circ
\]

平均角速度约：

\[
15^\circ/s
\]

### PROF1 — Smoothstep

\[
\lambda=p^2(3-2p)
\]

特点：

- 起始较缓；
- 中段较快；
- 末端较缓。

### PROF2~5

为固定节点的预设/优化轨迹，通过分段插值生成。

### PROF5 — POptD

节点为：

```text
progress : 0, 0.1667, 0.3333, 0.5, 0.6667, 0.8333, 1
lambda   : 0, 0.1029, 0.21985, 0.33117, 0.53781, 0.78641, 1
```

在 `TIME=6 s` 时大致对应：

| 时间 | lambda | 倾转角 |
|---:|---:|---:|
| 0 s | 0.0000 | 0.0° |
| 1 s | 0.1029 | 9.3° |
| 2 s | 0.2199 | 19.8° |
| 3 s | 0.3312 | 29.8° |
| 4 s | 0.5378 | 48.4° |
| 5 s | 0.7864 | 70.8° |
| 6 s | 1.0000 | 90.0° |

---

## 15. 参数之间最重要的逻辑关系

### 15.1 控制器权重

核心：

\[
MCW+FWW=1
\]

当前三轴均按互补权重转移：

```text
Pitch:
    MC Pitch × MCW
    FW Pitch × FWW

Roll:
    MC Roll × MCW
    FW Roll × FWW

Yaw:
    MC Yaw × MCW
    FW Yaw × FWW
```

其中：

- `MODE` 决定 `MCW/FWW`；
- `PROF` 决定真实倾转轨迹；
- 两者原则上独立。

### 15.2 TD3 与 MODE3

`PROF=8` 只负责决定：

```text
tilt lambda
```

`MODE=3` 则根据：

```text
Velocity + Current Tilt
```

通过 Mamdani FIS 生成：

```text
FWW
MCW = 1 - FWW
```

因此：

```text
PROF8 + MODE3
```

表示：

```text
TD3 决定倾转角
+
FIS 决定控制器分权
```

不是同一个控制器。

---

## 16. 参数快速索引

### 全局/实验选择

```text
Q_TILT_DCPT_EN
Q_TILT_DCPT_TIME
Q_TILT_DCPT_HNDT
Q_TILT_DCPT_MODE
Q_TILT_DCPT_PROF
```

### 高度/Pitch

```text
Q_TILT_DCPT_ALTP
Q_TILT_DCPT_ALTD
Q_TILT_DCPT_AMAX
Q_TILT_DCPT_FWAP
Q_TILT_DCPT_FWAD
Q_TILT_DCPT_PMAX
```

### 升力/推力/终端预测

```text
Q_TILT_DCPT_VLFT
Q_TILT_DCPT_CREG
Q_TILT_DCPT_LMAX
Q_TILT_DCPT_TFLT
Q_TILT_DCPT_TWIN
Q_TILT_DCPT_TGN
```

### Yaw

```text
Q_TILT_DCPT_YHP
Q_TILT_DCPT_YRM
Q_TILT_DCPT_YMAX

Q_TILT_DCPT_YAWP   # Legacy
Q_TILT_DCPT_YAWD   # Legacy
```

### Hard Switch

```text
Q_TILT_DCPT_SWLO
Q_TILT_DCPT_SWHI
Q_TILT_DCPT_SWMD
Q_TILT_DCPT_SWPK
Q_TILT_DCPT_SWKT
```

### Fake NMPC

```text
Q_TILT_DCPT_NPIT
Q_TILT_DCPT_NMAX
```

### TD3 Online

```text
Q_TILT_DCPT_TD3S
Q_TILT_DCPT_TD3V
Q_TILT_DCPT_TD3F
Q_TILT_DCPT_VEXP
Q_TILT_DCPT_EHF
Q_TILT_DCPT_EHL
```

### TD3 Shadow

```text
Q_TILT_DCPT_TDSH
Q_TILT_DCPT_TDSA
```

---

## 17. 实机使用时最容易混淆的几点

### 17.1 `TIME=6` 不等于手工把 `TD3S` 改 5 倍

正确：

```text
Q_TILT_DCPT_TIME = 6
Q_TILT_DCPT_TD3S = 0.084
```

代码自动得到：

```text
time_scale = 30/6 = 5
effective TD3S = 0.420
```

不要再手工设置：

```text
TD3S = 0.420
```

否则会被再次乘 5。

### 17.2 Shadow 不控制飞机

即使 `DTSH.Lam` 和真实 `TiltT` 完全不同，也不会改变真实飞行。

### 17.3 AP 官方组目前没有 Shadow

```text
DCPT_EN=0
```

时 Shadow 调度不会运行。

### 17.4 AUTO 也能触发 DCPTilt

决定因素是：

```text
是否发生 VTOL→FW 前向转场
```

而不是当前模式名是不是 FBWA。

### 17.5 FW→VTOL 始终是 AP 官方逻辑

无论：

```text
PROF=0
PROF=5
PROF=8
```

反向转场都不会使用自定义 DCPTilt。

---

## 18. 关于 ArduPilot 原生固定翼 Yaw PID

当前固定翼 Yaw 已调用：

```text
ArduPlane yawController.get_rate_out()
```

因此真正的固定翼 Yaw Rate PID 增益来自 ArduPilot 原生参数，而不是 `Q_TILT_DCPT_*`。

自定义外层参数只负责：

```text
Q_TILT_DCPT_YHP
    航向误差 → 目标 Yaw Rate

Q_TILT_DCPT_YRM
    限制目标 Yaw Rate
```

实际舵面 PID 动态继续由 ArduPilot 原生 Yaw Rate Controller 负责。

---

## 19. 当前建议的实验记录原则

为保证论文和实机数据可追溯，建议每组实验至少记录：

```text
Q_TILT_DCPT_EN
Q_TILT_DCPT_MODE
Q_TILT_DCPT_PROF
Q_TILT_DCPT_TIME

Q_TILT_DCPT_FWAP
Q_TILT_DCPT_FWAD
Q_TILT_DCPT_TWIN
Q_TILT_DCPT_TGN

Q_TILT_DCPT_TD3S
Q_TILT_DCPT_TD3V
Q_TILT_DCPT_TD3F
Q_TILT_DCPT_VEXP
Q_TILT_DCPT_EHF
Q_TILT_DCPT_EHL

Q_TILT_DCPT_TDSH
Q_TILT_DCPT_TDSA
```

对 Shadow/TD3 实验同时保留：

```text
RLT
RLSH
DTSH
DCPT
DCPA
DCPW
DCPZ
DCPX
DCPY
DCPR
DCPH
```

这样后续可以同时分析：

- 倾转轨迹；
- 控制权重；
- 姿态；
- 高度；
- 油门；
- TD3 Actor 输出；
- Shadow TD3 假想轨迹；
- 网络实时计算负载。

---

## 20. 嵌入式 TD3 实时性能指标

除了飞行控制效果，本项目还需要评估 TD3 Actor 在飞控端的实时计算性能。

在线 TD3 和 Shadow TD3 都以：

\[
20\ Hz
\]

运行，因此每次策略计算的理论时间预算为：

\[
T_{budget}=50\,ms=50\,000\,\mu s
\]

### 20.1 在线 TD3：`RLT`

当：

```text
Q_TILT_DCPT_PROF = 6 / 7 / 8
```

在线 TD3 会记录：

```text
RLT.TimeUS
RLT.ActorUS
RLT.ProjUS
RLT.TotalUS
RLT.Miss
RLT.Seq
```

含义：

| 字段 | 单位 | 含义 |
|---|---:|---|
| `TimeUS` | us | 本条日志时间戳 |
| `ActorUS` | us | 3-64-64-1 TD3 Actor 前向推理时间 |
| `ProjUS` | us | Actor 输出经过时间放缩、倾转速率映射、lambda 积分并进入真实倾转输出路径所需时间 |
| `TotalUS` | us | 一次在线 TD3 更新的总计算时间 |
| `Miss` | 0/1 | `TotalUS > 50000 us` 时为 `1`，表示超过 20 Hz 实时预算 |
| `Seq` | — | TD3 更新序号 |

### 20.2 Shadow TD3：`RLSH`

当：

```text
Q_TILT_DCPT_EN   = 1
Q_TILT_DCPT_PROF = 0~5
Q_TILT_DCPT_TDSH = 1
```

Shadow TD3 会记录：

```text
RLSH.TimeUS
RLSH.ActorUS
RLSH.TotalUS
RLSH.PeriodUS
RLSH.JitUS
RLSH.Miss
RLSH.Seq
```

含义：

| 字段 | 单位 | 含义 |
|---|---:|---|
| `ActorUS` | us | Actor 网络前向推理时间 |
| `TotalUS` | us | Shadow 一次完整更新总耗时，包括观测构造、Actor 推理和私有 lambda 更新；不包含真实执行器控制 |
| `PeriodUS` | us | 相邻 Shadow 更新之间的实际周期 |
| `JitUS` | us | 相对于理论 50000 us 周期的抖动 |
| `Miss` | 0/1 | 单次计算是否超过 50000 us 实时预算 |
| `Seq` | — | Shadow 更新序号 |

### 20.3 建议论文中统计的性能指标

对于每一次实验，至少统计：

#### Actor 推理耗时

```text
ActorUS mean
ActorUS median
ActorUS P95
ActorUS P99
ActorUS max
```

其中 P95/P99 比单独看平均值更能体现飞控端的最坏情况性能。

#### 总计算耗时

```text
TotalUS mean
TotalUS median
TotalUS P95
TotalUS P99
TotalUS max
```

#### CPU 时间预算占用率

定义：

\[
U=
\frac{TotalUS}{50000}
\times100\%
\]

例如：

```text
TotalUS = 500 us
```

则：

\[
U=1\%
\]

注意这里是 **20 Hz TD3 任务自身的时间预算占用率**，不是整颗飞控 CPU 的全局 CPU 使用率。

#### Deadline Miss Rate

\[
R_{miss}
=
\frac{\sum Miss}{N}
\times100\%
\]

理想情况：

```text
Miss = 0 for all samples
Rmiss = 0 %
```

#### Shadow 调度周期

理论：

\[
PeriodUS=50000
\]

可统计：

```text
PeriodUS mean
PeriodUS min
PeriodUS max
```

实际运行频率近似：

\[
f_{real}
=
\frac{10^6}{PeriodUS}
\]

#### 调度抖动

理论：

\[
JitUS=PeriodUS-50000
\]

建议统计：

```text
abs(JitUS) mean
abs(JitUS) P95
abs(JitUS) P99
abs(JitUS) max
```

### 20.4 6 s 实验样本数量

TD3 运行频率为 20 Hz，因此一次完整 6 s 转场理论上约有：

\[
6\times20=120
\]

个 TD3 样本。

因此一轮 6 s 实验足以获得基本的平均值、P95/P99 和最大值统计。

如果论文需要更可靠的最坏执行时间（WCET）证据，应将多次实验的 `RLT/RLSH` 样本合并统计，而不是只依赖单次约 120 个样本。

### 20.5 在线 TD3 与 Shadow TD3 的性能指标如何对应

网络结构完全相同时：

```text
Online PROF8 ActorUS
```

和：

```text
Shadow TD3C ActorUS
```

可以直接用于比较同一个 Actor 在两种运行方式下的前向推理耗时。

但是：

```text
RLT.TotalUS
```

和：

```text
RLSH.TotalUS
```

定义不完全相同，因此论文中如果要比较网络本身的嵌入式计算成本，优先比较：

```text
ActorUS
```

如果要说明整个在线控制算法是否满足实时性，则看：

```text
RLT.TotalUS
RLT.Miss
```

如果要说明后台部署是否会造成明显实时负担，则看：

```text
RLSH.TotalUS
RLSH.PeriodUS
RLSH.JitUS
RLSH.Miss
```

---

## 21. BIN 日志中如何查看本次实验使用的参数

ArduPilot DataFlash `.BIN` 日志会记录参数消息。

在日志中对应的消息类型是：

```text
PARM
```

因此，要确认某一份 BIN 实际使用了什么 `Q_TILT_DCPT_*` 参数，应优先查看：

```text
PARM.Name
PARM.Value
```

而不是只依赖实验 JSON 文件。

这是因为 BIN 中的 `PARM` 才代表该次飞行实际加载到飞控参数系统中的值。

### 21.1 推荐检查的参数

每次实验至少核对：

```text
Q_TILT_DCPT_EN
Q_TILT_DCPT_MODE
Q_TILT_DCPT_PROF
Q_TILT_DCPT_TIME

Q_TILT_DCPT_FWAP
Q_TILT_DCPT_FWAD
Q_TILT_DCPT_TWIN
Q_TILT_DCPT_TGN

Q_TILT_DCPT_TD3S
Q_TILT_DCPT_TD3V
Q_TILT_DCPT_TD3F
Q_TILT_DCPT_VEXP
Q_TILT_DCPT_EHF
Q_TILT_DCPT_EHL

Q_TILT_DCPT_TDSH
Q_TILT_DCPT_TDSA
```

### 21.2 用 `mavlogdump.py` 查看

如果系统已安装 `pymavlink`，可以直接：

```bash
mavlogdump.py --types PARM 00000335.BIN
```

只筛选 DCPTilt 参数：

```bash
mavlogdump.py --types PARM 00000335.BIN | grep Q_TILT_DCPT
```

如果 `mavlogdump.py` 不在 PATH 中，可尝试：

```bash
python3 -m pymavlink.tools.mavlogdump \
    --types PARM \
    00000335.BIN | grep Q_TILT_DCPT
```

输出中会看到类似：

```text
PARM {Name : Q_TILT_DCPT_MODE, Value : 3}
PARM {Name : Q_TILT_DCPT_PROF, Value : 8}
PARM {Name : Q_TILT_DCPT_TIME, Value : 6}
```

因此可以直接确认：

```text
这份 BIN 到底是哪一种实验
```

### 21.3 用 MAVExplorer 查看

打开：

```bash
MAVExplorer.py 00000335.BIN
```

在日志消息中查找：

```text
PARM
```

然后根据：

```text
Name
Value
```

确认参数值。

如果只需要确认某几个参数，命令行 `mavlogdump.py + grep` 通常比在图形界面中逐条寻找更快。

### 21.4 在日志分析软件中查看

使用 Mission Planner、MAVExplorer 或其他支持 ArduPilot DataFlash 的日志分析工具时：

1. 打开 `.BIN`；
2. 找到消息类型 `PARM`；
3. 查看 `Name`；
4. 搜索 `Q_TILT_DCPT_`；
5. 对应的 `Value` 即为飞控记录的参数值。

参数不是在：

```text
ATT
GPS
POS
DCPT
```

这些飞行时序消息中查看，而是在：

```text
PARM
```

中查看。

---

## 22. 当前实验中最重要的日志消息速查

### 飞行状态

```text
ATT     姿态 Roll/Pitch/Yaw
POS     位置/高度
GPS     GPS 位置和速度
```

### DCPTilt 主控制

```text
DCPT    转场进度、目标倾转角等核心状态
DCPA    控制器/分配相关状态
DCPW    MCW/FWW 权重
DCPZ    高度控制
DCPX    纵向/Pitch
DCPY    Yaw
DCPR    Roll
DCPH    转场结束后的 handover/航向状态
```

### 在线 TD3

```text
DCTD    TD3 Actor 主要状态
DCTV    TD3 速度观测
DCTE    TD3 高度误差观测
RLT     在线 TD3 嵌入式实时性能
```

### Shadow TD3

```text
DTSH    Shadow Actor 状态和假想 lambda
RLSH    Shadow 嵌入式实时性能
```

### 参数

```text
PARM    本次日志对应的飞控参数
```

因此，一份完整 TD3/DCPT 实验日志通常可以按以下顺序检查：

```text
1. PARM
   ↓
确认实验配置

2. DCPT / DCPW
   ↓
确认倾转轨迹和 MC/FW 权重

3. ATT / POS / DCPZ / DCPX
   ↓
评估姿态和高度性能

4. DCTD/DCTV/DCTE 或 DTSH
   ↓
评估 TD3 行为

5. RLT 或 RLSH
   ↓
评估嵌入式实时计算性能
```

