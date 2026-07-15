# Hero 自瞄 MPC 方案 B：电控对接文档

> **历史版本提示：** 本文记录的是 MPC-B v1.0 的 25B 下发方案，已不再用于当前联调。当前协议请以 [`MPC上下位机协议_v1.1.md`](MPC上下位机协议_v1.1.md) 为准：上位机下发 17B，不传输 `pitch_vel/pitch_acc`；下位机反馈仍为 25B。

> 适用项目：`sp_vision_25-main`（视觉上位机）与 `hero_rtos-master`（电控下位机）  
> 协议版本：MPC-B v1.0  
> 更新日期：2026-07-11

## 1. 重要警告

视觉上位机已经切换为方案 B，下位机仍按旧协议解析。在电控完成本文件中的接收修改前，**禁止将新版上位机直接连接实车运行**。

旧下位机会把方案 B 中的 `yaw_vel` 误认为 `pitch`，可能造成云台突然运动：

| 数据偏移 | 旧下位机含义 | 方案 B 含义 |
| ---: | --- | --- |
| `DATA[5..8]` | `pitch` | `yaw_vel` |
| `DATA[9..12]` | `roll / fire` | `yaw_acc` |

联调前必须确保上下位机协议版本一致，并在无弹、摩擦轮关闭、云台限流条件下进行首次测试。

---

## 2. 对接目标

视觉 MPC 每 10 ms 规划并下发一个控制点：

```text
目标位置：yaw、pitch
目标速度：yaw_vel、pitch_vel
目标加速度：yaw_acc、pitch_acc
控制状态：mode
```

电控需要完成：

1. 按本协议解析 25 字节 Payload。
2. 将目标位置接入角度控制。
3. 将目标速度作为速度前馈使用。
4. 将目标加速度作为加速度/力矩前馈使用。
5. 按 `mode` 处理视觉控制和开火请求。
6. 增加通信超时和异常包保护。

---

## 3. 下发帧总览

### 3.1 整帧结构

```text
┌────────┬────────┬──────┬──────┬───────────────────────┬────┬────┐
│ HEAD   │ D_ADDR │ ID   │ LEN  │ Payload               │ SC │ AC │
│ 1 byte │ 1 byte │ 1 B  │ 1 B  │ 25 bytes              │ 1B │ 1B │
└────────┴────────┴──────┴──────┴───────────────────────┴────┴────┘

总长度 = 4 + 25 + 2 = 31 bytes
```

固定值：

| 字段 | 类型 | 值 | 说明 |
| --- | --- | ---: | --- |
| `HEAD` | `uint8_t` | `0xFF` | 帧头 |
| `D_ADDR` | `uint8_t` | `0x02` | 保持现有机器人适配值 |
| `ID` | `uint8_t` | `0x20` | 云台控制包 |
| `LEN` | `uint8_t` | `25 / 0x19` | Payload 长度 |
| `SC` | `uint8_t` | 计算值 | 累加和校验 |
| `AC` | `uint8_t` | 计算值 | 二次累加校验 |

> 下位机枚举中虽然存在 `HERO = 0x06`，但当前视觉适配一直发送 `D_ADDR = 0x02`，且旧接收端未检查地址。本次为避免扩大变更范围，继续使用 `0x02`。若电控增加地址过滤，当前应接受 `0x02`。

### 3.2 Payload 定义

所有 `int32_t` 均采用 STM32/PC 常用的**小端序二进制补码**。

| Payload 偏移 | 长度 | 类型 | 字段 | 线上单位 |
| ---: | ---: | --- | --- | --- |
| `0` | 1 | `uint8_t` | `mode` | 枚举值 |
| `1..4` | 4 | `int32_t` | `yaw` | `degree × 1000` |
| `5..8` | 4 | `int32_t` | `yaw_vel` | `degree/s × 1000` |
| `9..12` | 4 | `int32_t` | `yaw_acc` | `degree/s² × 1000` |
| `13..16` | 4 | `int32_t` | `pitch` | `degree × 1000` |
| `17..20` | 4 | `int32_t` | `pitch_vel` | `degree/s × 1000` |
| `21..24` | 4 | `int32_t` | `pitch_acc` | `degree/s² × 1000` |

### 3.3 mode 定义

| mode | 含义 | 云台 | 开火 |
| ---: | --- | --- | --- |
| `0` | 视觉不控制/无有效目标 | 不更新视觉目标或安全保持 | 禁止视觉开火 |
| `1` | 视觉控制，有有效目标 | 使用六个 MPC 控制量 | 不开火 |
| `2` | 视觉控制并请求开火 | 使用六个 MPC 控制量 | 产生开火请求 |

`mode=2` 仅表示视觉提出开火请求，电控仍应检查操作手授权、摩擦轮、裁判系统、热量、通信状态等安全条件。

---

## 4. 数值编码与坐标方向

### 4.1 编码规则

视觉 Planner 内部使用 `rad、rad/s、rad/s²`，发送前统一转换为：

```text
degree × 1000
degree/s × 1000
degree/s² × 1000
```

示例：

| 物理量 | 实际值 | 线上 `int32_t` |
| --- | ---: | ---: |
| yaw | `1.5°` | `1500` |
| yaw_vel | `-20°/s` | `-20000` |
| yaw_acc | `100°/s²` | `100000` |

下位机解析：

```c
float value = raw_int32 / 1000.0f;
```

### 4.2 下位机坐标适配

根据 `hero_rtos-master` 现有接收代码，yaw 命令需要取负，pitch 命令保持原符号。方案 B 必须让同一轴的位置、速度、加速度使用相同符号：

```c
yaw       = -raw_yaw       / 1000.0f;
yaw_vel   = -raw_yaw_vel   / 1000.0f;
yaw_acc   = -raw_yaw_acc   / 1000.0f;

pitch     =  raw_pitch     / 1000.0f;
pitch_vel =  raw_pitch_vel / 1000.0f;
pitch_acc =  raw_pitch_acc / 1000.0f;
```

禁止只对 yaw 角度取负，却不处理 yaw 速度和加速度。

---

## 5. 电控接收端修改建议

### 5.1 主要文件

```text
hero_rtos-master/USER/Task/Transmission/transmission_task.c
hero_rtos-master/USER/rm_task.h
hero_rtos-master/USER/Task/Cmd/cmd_task.c
hero_rtos-master/USER/Task/Gimbal/gimbal_task.c
```

### 5.2 安全读取小端 int32

不建议直接使用未对齐的 `int32_t *` 强制转换。建议提供统一读取函数：

```c
static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t value =
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
    return (int32_t)value;
}
```

### 5.3 GIMBAL 包解析

原来的 `case GIMBAL` 应按以下字段读取：

```c
case GIMBAL:
{
    trans_fdb.vision_mode = rpy_rx_data.DATA[0];

    trans_fdb.yaw       = -read_i32_le(&rpy_rx_data.DATA[1])  / 1000.0f;
    trans_fdb.yaw_vel   = -read_i32_le(&rpy_rx_data.DATA[5])  / 1000.0f;
    trans_fdb.yaw_acc   = -read_i32_le(&rpy_rx_data.DATA[9])  / 1000.0f;

    trans_fdb.pitch     =  read_i32_le(&rpy_rx_data.DATA[13]) / 1000.0f;
    trans_fdb.pitch_vel =  read_i32_le(&rpy_rx_data.DATA[17]) / 1000.0f;
    trans_fdb.pitch_acc =  read_i32_le(&rpy_rx_data.DATA[21]) / 1000.0f;

    vision_last_rx_ms = dwt_get_time_ms();
    break;
}
```

同时检查：

```c
HEAD == 0xFF
D_ADDR == 0x02
ID == 0x20
LEN == 25
SC、AC 正确
mode <= 2
```

### 5.4 消息结构扩展

`trans_fdb_msg` 至少增加：

```c
uint8_t vision_mode;
float yaw;
float yaw_vel;
float yaw_acc;
float pitch;
float pitch_vel;
float pitch_acc;
```

`gimbal_cmd_msg` 至少增加：

```c
float yaw_vel;
float yaw_acc;
float pitch_vel;
float pitch_acc;
```

视觉的 `mode=0/1/2` 不应直接复用 `gimbal_mode_e`。建议保留两套状态：

```text
gimbal_mode_e：操作手/机器人状态机，决定是否进入 GIMBAL_AUTO
vision_mode：视觉包状态，决定目标是否有效以及是否请求开火
```

---

## 6. 云台控制接入建议

当前电控是角度环和速度环串级 PID。MPC 数据建议这样接入：

```text
角度 PID 输出 + 视觉目标速度
              ↓
          速度目标
              ↓
速度 PID 输出 + Kff_acc × 视觉目标加速度
              ↓
          电机电流
```

伪代码：

```c
float speed_ref = pid_calculate(
    pid_angle,
    current_angle,
    target_angle
) + target_velocity;

float current_ref = pid_calculate(
    pid_speed,
    current_velocity,
    speed_ref
) + acceleration_feedforward_gain * target_acceleration;
```

要求：

- yaw 使用 `yaw/yaw_vel/yaw_acc`。
- pitch 使用 `pitch/pitch_vel/pitch_acc`。
- 前馈项必须经过速度、电流和机械限位保护。
- `acceleration_feedforward_gain` 由电控结合电机、负载和控制周期标定。
- 避免下位机再次对目标做强平滑，否则会抵消上位机 MPC 轨迹。

mode 行为建议：

```c
if (gimbal_mode == GIMBAL_AUTO)
{
    if (vision_mode == 1 || vision_mode == 2)
    {
        // 更新位置、速度、加速度目标
    }
    else
    {
        // 无有效视觉目标：禁止开火，前馈清零，安全保持/交还手控
    }
}
```

---

## 7. 开火与安全策略

视觉开火请求：

```c
bool vision_fire_request = (vision_mode == 2);
```

建议最终开火条件：

```text
vision_fire_request
AND 操作手允许自动瞄准
AND 云台处于 GIMBAL_AUTO
AND 摩擦轮正常
AND 裁判系统允许
AND 热量允许
AND 通信未超时
AND 数据包校验通过
```

失去任意条件都应立即取消视觉开火请求。不得因为曾经收到一帧 `mode=2` 而持续锁存开火。

---

## 8. 校验算法

上位机对 `HEAD、D_ADDR、ID、LEN、Payload` 共 29 字节逐字节计算：

```c
uint8_t sum = 0;
uint8_t add = 0;

for (size_t i = 0; i < 29; ++i)
{
    sum += frame[i];
    add += sum;
}

frame[29] = sum;  // SC
frame[30] = add;  // AC
```

下位机必须采用同样的逐字节顺序。注意这与当前下位机 `Check_Rpy()` 中“先把四个头字段一次性加入 add”的写法不同。

---

## 9. 通信超时保护

推荐参数：

```text
视觉下发频率：100 Hz
正常周期：10 ms
失联超时：50～100 ms
```

超时后立即：

```text
vision_mode = 0
视觉开火请求 = false
yaw_vel/yaw_acc = 0
pitch_vel/pitch_acc = 0
云台安全保持或交还操作手
```

错误地址、错误 ID、错误 LEN、错误校验、非法 mode 均不得刷新 `vision_last_rx_ms`。

---

## 10. 抓包测试向量

输入物理量：

```text
mode      = 1
yaw       = +1°
yaw_vel   = +2°/s
yaw_acc   = +3°/s²
pitch     = -1°
pitch_vel = -2°/s
pitch_acc = -3°/s²
```

上位机发送的完整 31 字节应为：

```text
FF 02 20 19 01
E8 03 00 00
D0 07 00 00
B8 0B 00 00
18 FC FF FF
30 F8 FF FF
48 F4 FF FF
32 24
```

其中：

```text
SC = 0x32
AC = 0x24
```

电控按坐标适配解析后应得到：

```text
vision_mode = 1
yaw       = -1°
yaw_vel   = -2°/s
yaw_acc   = -3°/s²
pitch     = -1°
pitch_vel = -2°/s
pitch_acc = -3°/s²
```

---

## 11. 联调顺序

### 阶段 A：仅验证通信

- [ ] 架空云台或断开电机输出。
- [ ] 下位机打印原始 31 字节。
- [ ] 使用测试向量核对字段偏移、负数、小端序、SC、AC。
- [ ] 验证 `mode=0/1/2`。

### 阶段 B：仅验证位置

- [ ] 禁止开火，摩擦轮关闭。
- [ ] 暂时令速度和加速度前馈系数为 0。
- [ ] 下发 yaw/pitch 小角度阶跃（建议 `±1°`）。
- [ ] 确认 yaw/pitch 正方向和限位。

### 阶段 C：加入速度前馈

- [ ] 加入 `yaw_vel/pitch_vel`。
- [ ] 从较小前馈权重开始。
- [ ] 记录目标角度、实际角度、目标速度、实际速度和电流。
- [ ] 检查是否抖动、超调或方向相反。

### 阶段 D：加入加速度前馈

- [ ] 加入 `yaw_acc/pitch_acc`。
- [ ] 从很小的加速度前馈系数开始。
- [ ] 设置电流、速度、加速度硬限幅。
- [ ] 检查换向瞬间是否出现电流冲击。

### 阶段 E：开火联调

- [ ] 先用指示灯/日志代替真实开火。
- [ ] 验证只有 `mode=2` 才产生请求。
- [ ] 验证 `mode=0/1`、通信超时和校验失败都立即撤销请求。
- [ ] 最后在安全条件下进行实弹测试。

---

## 12. 验收清单

### 通信

- [ ] 固定帧长 31 字节。
- [ ] `HEAD=FF、D_ADDR=02、ID=20、LEN=19`。
- [ ] 25 字节 Payload 偏移完全一致。
- [ ] 小端有符号 `int32_t` 解析正确。
- [ ] SC、AC 校验正确。
- [ ] 非法包不更新控制目标。

### 坐标与单位

- [ ] 上位机弧度制正确转换为度制 ×1000。
- [ ] yaw 三项统一取负。
- [ ] pitch 三项统一不取负。
- [ ] 正负角速度、加速度解析正确。

### 控制与安全

- [ ] 位置、速度、加速度均被控制器实际使用。
- [ ] 速度和加速度前馈有独立系数及限幅。
- [ ] pitch 机械限位和电机电流限幅有效。
- [ ] 模式切换时清理 PID/前馈历史。
- [ ] `mode=0/1` 禁止视觉开火。
- [ ] `mode=2` 仅生成安全链路中的开火请求。
- [ ] 50～100 ms 失联后退出视觉控制。
- [ ] 上下位机协议版本不一致时禁止上车。

---

## 13. 首阶段不修改的内容

为降低联调复杂度，第一阶段保持“下位机 → 上位机”反馈格式不变，上位机继续读取：

```text
DATA[1..4]：yaw
DATA[5..8]：pitch
```

待方案 B 下发、位置/速度/加速度前馈稳定后，再单独设计反馈协议，增加：

```text
mode、yaw_vel、pitch_vel、bullet_speed、bullet_count
```
