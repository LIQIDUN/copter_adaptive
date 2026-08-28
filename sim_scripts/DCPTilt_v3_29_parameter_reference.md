# DCPTilt v3.29 参数说明

> 对应版本：`v3.29_AP_YAW_RATE`  
> 参数来源：当前 `ArduPlane/tiltrotor.cpp` 中新增/修改的 `DCPT_*` 参数定义。  
> 完整参数名均带 ArduPilot Tiltrotor 前缀，因此源码中的 `DCPT_TIME` 在地面站中显示为 `Q_TILT_DCPT_TIME`。
>
> **注意：** 下表“范围”是代码中 `@Range` 声明的可设置范围，不等于“实机安全范围”。实机应先在 SITL、无桨台架和低风险飞行中逐步验证。

---

## 1. 总体开关、转场时间和实验选择

| 参数 | 默认值 | 范围/可选值 | 含义 | 建议 |
|---|---:|---|---|---|
| `Q_TILT_DCPT_EN` | `0` | `0/1` | 选择前向转场方式。`0`=ArduPilot 原生倾转转场；`1`=DCPTilt 自定义前向转场。选择在每次已解锁前向转场开始时锁存，中途修改不会切换当前转场。固定翼→VTOL 始终走 AP 原生返回逻辑。 | 做 DCPT 实验时设 `1`；对照组设 `0`。 |
| `Q_TILT_DCPT_TIME` | `30 s` | `1~120 s` | DCPT 前向转场总时长。达到该时间后主转场结束并进入固定翼。 | 当前实验常用 `30 s`。 |
| `Q_TILT_DCPT_HNDT` | `3 s` | `0~10 s` | DCPT 主转场结束后，倾转电机实际输出向固定翼油门需求平滑交接的时间。`0` 关闭该平滑。 | 一般保留默认，除非专门研究油门交接。 |
| `Q_TILT_DCPT_MODE` | `0` | `0:FUZZ` `1:SWITCH` `2:NMPC` `3:FIS` | 选择 **MC/FW 控制器权重分配策略**。它与倾转轨迹 `PROF` 独立。 | 做公平对比时只改 MODE，不改其它公共参数。 |
| `Q_TILT_DCPT_PROF` | `0` | `0:Linear` `1:Smoothstep` `2:POptA` `3:POptB` `4:POptC` `5:POptD` `6:TD3A` `7:TD3B` `8:TD3C` | 选择倾转角轨迹。0~5 为预设时序轨迹，6~8 为 20 Hz TD3 Actor 增量积分轨迹。 | 当前神经网络实验主要用 `8`。 |

### MODE 与 PROF 的关系

它们是两件不同的事：

- `MODE` 决定 **多旋翼控制器 MCW 和固定翼控制器 FWW 怎么分权**；
- `PROF` 决定 **短舱/电机倾转角怎么变化**。

原则上可以例如：

```text
MODE = FIS
PROF = Linear
```

也可以：

```text
MODE = FIS
PROF = TD3C
```

---

## 2. 公共高度/纵向控制参数

这组参数决定转场期间的定高，以及固定翼 Pitch 支路的目标生成。

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_ALTP` | `0.50` | `0~3` | `(m/s²)/m` | 高度误差到垂向加速度修正的 P 增益。飞机低于转场入口高度时，产生向上的加速度修正。 |
| `Q_TILT_DCPT_ALTD` | `0.80` | `0~5` | — | 垂向速度阻尼增益。抑制高度回正过程中的上冲/下冲。 |
| `Q_TILT_DCPT_AMAX` | `3.0` | `0.2~6` | `m/s²` | 公共高度环允许产生的最大绝对垂向加速度修正。 |
| `Q_TILT_DCPT_FWAP` | `2.0` | `0~10` | `deg/m` | 固定翼 Pitch 目标对高度误差的 P 增益。 |
| `Q_TILT_DCPT_FWAD` | `3.0` | `0~15` | `deg/(m/s)` | 固定翼 Pitch 目标对垂向速度的阻尼增益。 |
| `Q_TILT_DCPT_PMAX` | `15` | `3~30` | `deg` | DCPT 高度控制器允许生成的最大绝对固定翼 Pitch 目标。 |

固定翼纵向支路大致可以理解为：

\[
\theta_{FW,cmd}
=
K_{h}e_h-K_vV_z
\]

再由 ArduPlane 原生 `pitchController` 跟踪，并乘固定翼权重 `FWW`。

---

## 3. 机翼升力估计、总推力和终端高度修正

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_VLFT` | `22` | `5~40` | `m/s` | 简化机翼升力模型的参考速度。代码使用近似 `Lwing/mg = (V/VLFT)^2`，再由 `LMAX` 限幅。 |
| `Q_TILT_DCPT_CREG` | `0.12` | `0.03~0.5` | — | `sqrt(cos²(theta)+CREG²)` 中的正则项，避免大倾角时出现推力补偿奇异和高频放大。 |
| `Q_TILT_DCPT_LMAX` | `0.95` | `0.5~1.0` | — | 最大建模机翼升力占重力比例 `Lwing/(mg)`。低于 1 可以保留少量旋翼垂向余量。 |
| `Q_TILT_DCPT_TFLT` | `0.15` | `0~1` | `s` | 公共总推力指令的一阶滤波时间常数。`0` 表示关闭滤波。不会改变倾转角轨迹。 |
| `Q_TILT_DCPT_TWIN` | `0.25` | `0.05~0.50` | 转场时长比例 | 终端高度预测器在最后多少比例的转场时间内逐步介入。 |
| `Q_TILT_DCPT_TGN` | `1.0` | `0~3` | — | 终端预测高度误差的增益。`0`=关闭终端修正，`1`=按完整预测误差加入。 |

终端预测器的核心思想是：

\[
h_{pred}=h+V_z t_{remain}
\]

在转场末段提前考虑“按当前垂向速度继续飞，到 30 s 时会到哪里”，减少末端高度过冲。

---

## 4. Yaw 参数 —— v3.29 当前结构

v3.29 已经不再使用我们早期自写的“航向误差 PD 直接打方向舵”。

现在结构是：

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

多旋翼侧同时保持：

```text
AP Multicopter Yaw Controller
    ↓
× MCW
```

因此 Yaw 也成为真正的互补控制权分配。

### 当前有效的 Yaw 参数

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_YHP` | `1.0` | `0~4` | `1/s` | 航向外环 P。把航向误差（deg）转换成固定翼目标 Yaw Rate（deg/s）。 |
| `Q_TILT_DCPT_YRM` | `20` | `1~60` | `deg/s` | 固定翼 Yaw 支路允许的最大绝对目标偏航角速度。 |
| `Q_TILT_DCPT_YMAX` | `3000` | `0~4500` | AP 舵面 scaled output | 固定翼 Yaw 原始方向舵输出的绝对限幅，之后才乘 `FWW`。`4500` 代表满舵。 |

例如默认情况下：

\[
e_\psi=8^\circ,\quad YHP=1
\]

则外环先要求：

\[
r_d=8^\circ/s
\]

只要未超过 `YRM=20 deg/s`，就把该目标交给 ArduPlane 原生 Yaw Rate PID。

### Roll 协调转弯

v3.29 还复用了 ArduPilot Tiltrotor 原生的 Yaw target 更新逻辑：

- 小 Roll 需求时，基本保持当前共同航向目标；
- 有有效空速并且 `|nav_roll| > 10°` 时，Yaw target 会按固定翼协调转弯所需的航向变化率推进；
- MC 和 FW 两个 Yaw 控制器追踪同一个更新后的 Yaw target。

这比“固定航向目标 + 额外硬加一个 Roll→Yaw Rate”更一致。

### 已保留但 **v3.29 不再使用** 的旧参数

| 参数 | 默认值 | 范围 | 状态 |
|---|---:|---:|---|
| `Q_TILT_DCPT_YAWP` | `100` | `0~400` | **Legacy，仅为参数兼容保留；v3.29 FW Yaw 不再使用。** |
| `Q_TILT_DCPT_YAWD` | `150` | `0~500` | **Legacy，仅为参数兼容保留；v3.29 FW Yaw 不再使用。** |

---

## 5. Hard Switch 模式专用参数

只在 `Q_TILT_DCPT_MODE=1` 时主要生效。

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_SWLO` | `6` | `0~40` | `m/s` | 低速阈值。低于它：`MCW=1, FWW=0`。 |
| `Q_TILT_DCPT_SWHI` | `17` | `0.5~50` | `m/s` | 高速阈值。高于它：`MCW=0, FWW=1`。 |
| `Q_TILT_DCPT_SWMD` | `0.50` | `0~1` | — | SWLO~SWHI 中间区的固定翼权重。`MCW=1-SWMD`。 |
| `Q_TILT_DCPT_SWPK` | `4.0` | `0~10` | `deg` | FWW 正向阶跃时附加的固定翼 Pitch 短暂 nose-down kick 增益。`0` 关闭。 |
| `Q_TILT_DCPT_SWKT` | `1.2` | `0.2~3` | `s` | 上述 Pitch kick 的平滑衰减时间。 |

当前 Hard Switch 逻辑可简化理解为：

```text
V <= SWLO      : MCW=1,       FWW=0
SWLO < V < SWHI: MCW=1-SWMD,  FWW=SWMD
V >= SWHI      : MCW=0,       FWW=1
```

---

## 6. “Fake NMPC” 模式专用参数

只在 `Q_TILT_DCPT_MODE=2` 时使用。

MODE 2 的控制权重本身复用 MODE 0 的 FUZZ 分配；它额外加入的是一个早期 Pitch 瞬态。

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_NPIT` | `35` | `0~80` | `deg` | 早期 `MCW*sin(tilt)` Pitch bias 的增益。 |
| `Q_TILT_DCPT_NMAX` | `4` | `0~20` | `deg` | 上述额外 Pitch bias 的最大值。`0` 可以关闭该瞬态。 |

该额外 Pitch bias 只在转场早期存在，并在约 35% 转场进度前平滑衰减到 0。

---

## 7. TD3 Actor 参数

只对 `PROF=6/7/8`（TD3A/B/C）有意义。

Actor 运行频率固定为 **20 Hz**，每 50 ms 更新一次。

### 7.1 Actor 输出到倾转速率

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_TD3S` | `0.084` | `0~1` | `1/s` | Actor Sigmoid 输出到归一化倾转速率的比例。`lambda_dot = TD3S * actor_output`。 |

离散积分近似为：

\[
\lambda_{k+1}
=
\lambda_k
+
TD3S\cdot a_k\cdot0.05
\]

其中 `a_k` 为 Actor 输出。

### 7.2 TD3 速度观测

| 参数 | 默认值 | 范围/可选值 | 单位 | 含义 |
|---|---:|---|---|---|
| `Q_TILT_DCPT_TD3V` | `1` | `0:LegacyStrategy` `1:AirspeedAuto` `2:Groundspeed` `3:NED3D` | — | 选择 Actor 的速度输入来源。 |
| `Q_TILT_DCPT_TD3F` | `25` | `5~50` | `m/s` | 当前飞机典型平飞速度，用来把当前平台速度缩放到训练飞机的 25 m/s 尺度。 |
| `Q_TILT_DCPT_VEXP` | `1.0` | `1~2` | — | TD3 速度归一化指数。`1`=线性速度；`2`=诊断性平方速度/动压型缩放。 |

当前代码的速度归一化可概括为：

\[
V_n
=
1.5
\left(
\frac{V_{used}}{V_{flat}}
\right)^{VEXP}
\]

其中：

- `Vused` 由 `TD3V` 决定；
- `Vflat = TD3F`。

### 7.3 TD3 高度误差观测诊断

| 参数 | 默认值 | 范围 | 单位 | 含义 |
|---|---:|---:|---|---|
| `Q_TILT_DCPT_EHF` | `0` | `0~60` | `s` | 在指定转场时刻冻结 Actor 使用的 Eh，高度控制器仍使用真实误差。`0`=关闭。仅诊断。 |
| `Q_TILT_DCPT_EHL` | `0.5` | `0~2` | `m` | 只限制送给 Actor 的 Eh 绝对值。`0`=不裁剪。不会限制真实高度控制器误差。 |

TD3 中我们使用的 Eh 符号是：

\[
Eh=h_{ref}-h_{actual}
\]

因此：

- `Eh > 0`：飞机低于参考高度；
- `Eh < 0`：飞机高于参考高度。

---

## 8. 当前常用 TD3C 实验配置

代码默认值并不完全等于我们最近用于复现实验的参数。

目前较常用的一组配置是：

```text
Q_TILT_DCPT_EN    1
Q_TILT_DCPT_PROF  8
Q_TILT_DCPT_TIME  30

Q_TILT_DCPT_TD3V  1
Q_TILT_DCPT_TD3F  18
Q_TILT_DCPT_TD3S  0.084
Q_TILT_DCPT_VEXP  2

Q_TILT_DCPT_EHF   0
Q_TILT_DCPT_EHL   0
```

其中：

- `PROF=8`：Actor C；
- `TD3V=1`：优先使用空速；
- `TD3F=18`：按当前约 2 kg/SITL 平台平飞速度做尺度映射；
- `VEXP=2`：当前实验采用的诊断性平方速度缩放；
- `EHF=0`：不冻结 Eh；
- `EHL=0`：当前基线不裁剪 Actor Eh。

**这是一组研究实验配置，不应理解成所有真实飞机都适用的安全默认值。**

---

## 9. 参数之间最重要的逻辑关系

### 9.1 控制器权重

核心思想始终是：

\[
MCW+FWW=1
\]

当前三轴均按这一结构做控制权转移：

```text
Pitch:
    MC Pitch × MCW
    FW Pitch × FWW

Roll:
    MC Roll  × MCW
    FW Roll  × FWW

Yaw:
    MC Yaw   × MCW
    FW Yaw   × FWW
```

其中：

- `MODE` 决定 MCW/FWW；
- `PROF` 决定倾转轨迹；
- 两者原则上互相独立。

### 9.2 v3.29 Yaw

当前 Yaw 不再使用：

```text
Yaw error → 手写 PD → Rudder
```

而是：

```text
Yaw error
    ↓ Q_TILT_DCPT_YHP
desired yaw rate
    ↓ Q_TILT_DCPT_YRM 限幅
AP 原生 fixed-wing yaw-rate PID
    ↓ Q_TILT_DCPT_YMAX 限幅
× FWW
    ↓
Rudder
```

因此如果后续需要调 Yaw 动态：

1. 先看 `Q_TILT_DCPT_YHP` 是否过强/过弱；
2. 再看 `Q_TILT_DCPT_YRM` 是否限制过紧；
3. 然后才调 **ArduPilot 原生 Yaw Rate PID 参数**；
4. `Q_TILT_DCPT_YAWP/YAWD` 在 v3.29 已经不起作用。

---

## 10. 参数快速索引

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

Q_TILT_DCPT_YAWP   # Legacy，v3.29不使用
Q_TILT_DCPT_YAWD   # Legacy，v3.29不使用
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

### TD3

```text
Q_TILT_DCPT_TD3S
Q_TILT_DCPT_TD3V
Q_TILT_DCPT_TD3F
Q_TILT_DCPT_VEXP
Q_TILT_DCPT_EHF
Q_TILT_DCPT_EHL
```

---

## 11. 一个需要特别注意的点

v3.29 的 FW Yaw 已经调用 **ArduPlane 原生 `yawController.get_rate_out()`**。

因此真正的固定翼 Yaw Rate PID 增益来自 ArduPilot 自身的 Yaw Rate 控制器参数，而不是 `Q_TILT_DCPT_*` 参数。

这些原生 PID 参数没有在本文件的 `DCPT_*` 参数表中定义，所以本说明不擅自填写它们的具体范围。调参时应以你当前 **ArduPlane 4.5.7 实际参数表/地面站显示**为准。

我们自定义的两个 Yaw 外层参数只负责：

```text
Q_TILT_DCPT_YHP : 航向误差 → 目标Yaw Rate
Q_TILT_DCPT_YRM : 限制目标Yaw Rate
```

真正的舵面 PID 动态由 AP 原生 Yaw Rate Controller 负责。
