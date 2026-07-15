# STC32G144K 智能车固件

本工程基于逐飞 STC32G144K 库和 Keil C251 工具链，实现摄像头循迹、左右环岛、编码器测速、电机与舵机闭环、靶点识别、激光时序和 WiFi 调试。

## 快速开始

| 项目 | 当前配置 |
| --- | --- |
| Keil 工程 | `project/mdk/seekfree.uvproj` |
| 构建目标 | `stc32g144k246` |
| 芯片 | STC32G144K246 / AI8052U-32Bit Series |
| 业务入口 | `project/code/main.c` |
| 活跃中断文件 | `project/user/isr.c` |
| 图像尺寸 | 188 × 120 |
| 控制周期 | 15 ms |
| 发车按键 | `KEY1_PIN = IO_PB2` |
| WiFi 调试开关 | `SWITCH2_PIN = IO_PB1` |

`project/user/main.c` 没有加入当前 Keil 目标，仅作历史参考。`project/user/` 不能整个当成废弃目录，因为其中的 `isr.c` 仍在编译。

## 目录与核心文件

```text
libraries/                 逐飞底层库和设备驱动
project/code/              主要业务代码
project/user/isr.c         当前中断入口
project/mdk/               Keil 工程
project/mdk/out_file/      构建产物，不手工修改
```

| 文件 | 职责 |
| --- | --- |
| `project/code/main.c` | 硬件初始化、按键发车、WiFi 开关和图像主循环 |
| `project/code/image.c` | 阈值、参考列、左右边线、控制中线和目标检测调度 |
| `project/code/ring.c` | 左右环岛检测、状态转移和补线 |
| `project/code/pid.c` | 舵机 PID 和电机 PI |
| `project/code/motor.c` | 编码器、目标速度、差速和电机 PWM |
| `project/code/servo.c` | 舵机 PWM 初始化、闭环和安全限幅 |
| `project/code/laser.c` | 靶点检测和激光脉冲/冷却时序 |
| `project/code/assistant_debug.c` | 图像边界、状态叠加、示波器数据和在线调参 |
| `project/code/wifi_assistant.c` | WiFi 连接与调试助手会话 |

## 运行链路

### 图像与转向

```text
摄像头帧完成
  -> Image_OldStyle_Process()
  -> 灰度阈值与参考列
  -> left_edge_line[] / right_edge_line[]
  -> Ring() 环岛补线
  -> left_control_line[] / right_control_line[]
  -> mid_line[]
  -> Servo_Loop()
  -> 舵机 PWM
```

`Image_OldStyle_Process()` 在主循环中每帧执行，顺序为：

1. `otsuThreshold()` 计算大津阈值。
2. `get_reference_point()` 更新黑白和反光判定阈值。
3. `Search_reference_col()` 选择当前参考列。
4. `Search_line()` 搜索原始左右边线，并复制到控制线。
5. `Ring()` 按环岛状态修正控制线。
6. `Fitted_Midline()` 生成舵机使用的中线。
7. 条件允许时运行 `Target_find()`。

### 15 ms 控制任务

按下发车键后才启动 `TIM0_PIT`，回调中仅执行：

```c
Encoder_GetValue();
Servo_Loop();
Motor_Loop();
Laser_Task();
```

图像处理、WiFi 通信和格式化输出不应搬进该中断。

## 环岛状态机

| `current_step` | 状态 | 含义 |
| ---: | --- | --- |
| 0 | `RING_STEP_PRE_MEET` | 普通循线，等待环岛预判 |
| 1 | `RING_STEP_FIRST_MEETING` | 识别首次边线突变 |
| 2 | `RING_STEP_ENTER` | 进入环岛前的边线跟踪 |
| 3 | `RING_STEP_TURN` | 补线并转入环内 |
| 4 | `RING_STEP_IN` | 环内循线 |
| 5 | `RING_STEP_OUT` | 识别并转向出口 |
| 6 | `RING_STEP_STRAIGHTEN` | 出环后回正 |
| 7 | `RING_STEP_OVER` | 结束并复位到普通循线 |

状态 1–6 期间必须只锁定一个方向：`ring_l` 或 `ring_r`。方向标志冲突或丢失时，状态机会安全复位。

### 右环当前策略

右环使用固定补线和固定环内驻留逻辑：

- 右侧坐标使用 `RING_MIRROR_COL()` 从 188 像素宽度计算。
- `ENTER → TURN` 必须先在近端观测到 `midPoint >= 60`，使 `mid_under_flag` 置位。
- 在 `mid_under_flag` 已置位的前提下，`turn_flag` 成立或 `midPoint < 45` 才允许进入 `TURN`。
- 右环进入 `IN` 后，使用顶部列 `140` 向底部列 `0` 画固定左控制线。
- 环内累计 40 帧后进入 `OUT`。
- `OUT` 阶段恢复原有大角度补线，直到边线满足回正条件。

不再使用按行宽度学习、单边推算或上一帧中线退路，避免环内左右控制线跳变。

### 环岛调试顺序

1. 先确认 `ring_l` / `ring_r` 方向正确。
2. 记录 `current_step` 的连续帧序列，不只看一张截图。
3. 卡在 1：检查首次拐点、宽度和 85/94 行边线。
4. 卡在 2：检查 `midPoint`、`turn_flag`、第 110 行右边线以及 167 阈值。
5. 卡在 3：检查 `rightTopPoint` 和连续 3 帧确认。
6. 卡在 4：检查 50/60 行出口边线。
7. 一次只改一类检测条件、状态转移或补线策略。

## 关键配置

### 功能开关

| 宏 | 当前值 | 位置 |
| --- | ---: | --- |
| `TARGET_DETECT_ENABLE` | 1 | `project/code/image.c` |
| `ZEBRA_DETECT_ENABLE` | 0 | `project/code/image.c` |
| `IMAGE_COPY_ENABLE` | 0 | `project/code/image.c` |
| `RIGHT_RING_ENABLE` | 1 | `project/code/ring.c` |
| `ASSISTANT_DEBUG_ENABLE` | 1 | `project/code/assistant_debug.h` |

`current_step >= 2` 时跳过靶点检测，避免环岛黑区和断线造成误触发。

### 图像与舵机

| 参数 | 当前值 | 位置 |
| --- | ---: | --- |
| `SEARCH_IMAGE_W` | 188 | `project/code/image.h` |
| `SEARCH_IMAGE_H` | 120 | `project/code/image.h` |
| `PIXEL_OFFSET` | 2 | `project/code/image.h` |
| `STOP_ROW` | 8 | `project/code/image.h` |
| `controlReferenceLine` | 80 | `project/code/servo.h` |
| `Mid_Col` | 94 | `project/code/servo.h` |
| `reference_contrast_ratio` | 120 | `project/code/image.c` |
| `camera_exposure_time` | 35 | `project/code/image.c` |
| `SERVO_DUTY_MIN` | 690 | `project/code/servo.h` |
| `SERVO_DUTY_MID` | 850 | `project/code/servo.h` |
| `SERVO_DUTY_MAX` | 1040 | `project/code/servo.h` |

### 闭环参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `SERVO_KP_TURN_Q10` | 4500 | 转弯舵机 P |
| `SERVO_KP_STRAIGHT_Q10` | 2000 | 直道舵机 P |
| `SERVO_KD_Q10` | 200 | 舵机 D |
| `SERVO_KF_Q10` | 200 | 曲率前馈 |
| `MOTOR_KP_Q10` | 1024 | 电机 PI 的 P |
| `MOTOR_KI_Q10` | 150 | 电机 PI 的 I |

PID/PI 参数采用 Q10 定点格式，代码中的整数值等于实际系数乘以 1024。

### 电机与速度

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `MOTOR_FREQ` | 17000 | 电机 PWM 频率 |
| `MOTOR_MAX_LIMIT` | 7500 | 最终 PWM 限幅 |
| `MAX_SPEED` | 180 | 小转向目标速度 |
| `MIN_SPEED` | 165 | 大转向目标速度 |
| `BASE_TARGET_SPEED` | 170 | 默认目标速度 |

速度值的单位是 15 ms 窗口内的编码器脉冲数，不是 RPM。

## 靶点、激光与 WiFi

- `Target_find()` 在限定 ROI 内寻找靶点，并更新 `tar_x`、`tar_y` 和 `aim_ready_flag`。
- `Laser_Task()` 在 15 ms 任务中执行，当前发射 2 tick，随后冷却 34 tick。
- WiFi 仅在 `SWITCH2_PIN` 有效时启用。
- 网络名、密码、上位机 IP 和端口位于 `project/code/wifi_assistant.c`，不在文档中复制其实际值。
- 调试助手示波器当前输出中线误差、舵机误差、左右编码器以及右边线 50/60/70/80 行数据。

## 硬件连接摘要

| 模块 | 引脚或通道 |
| --- | --- |
| 左电机方向 / PWM | `IO_P52` / `PWMD_CH1_P50` |
| 右电机方向 / PWM | `IO_P53` / `PWMD_CH2_P51` |
| 左编码器 | `PWMC_ENCODER`，读取为 `encoder_data_l` |
| 右编码器 | `PWMA_ENCODER`，读取后取负为 `encoder_data_r` |
| 舵机 PWM | `PWME_CH2P_PA2` |
| 激光 | `IO_P67` |
| 发车按键 | `IO_PB2` |
| WiFi 调试开关 | `IO_PB1` |

以上左右极性与当前代码一致。更换电机、编码器或接线后，必须在车轮离地和低输出条件下重新确认方向。

## 构建与下载

### Keil 图形界面

1. 打开 `project/mdk/seekfree.uvproj`。
2. 选择 `stc32g144k246`。
3. 执行 Build/Rebuild。
4. 检查 `project/mdk/out_file/SEEKFREE.build_log.htm`。
5. 需要下载时使用 `project/mdk/out_file/SEEKFREE.hex`。

### 命令行

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'project\mdk\seekfree.uvproj' -t 'stc32g144k246'
```

2026-07-14 的最近一次 Keil C251 构建结果：

```text
0 Error(s), 0 Warning(s)
data=8.6
edata+hdata=53324
xdata=33375
const=14020
code=127844
```

构建成功只证明编译与链接通过，不代表环岛、舵机、电机或激光已完成实车验证。

## 调试与安全约束

- 保持图像处理在主循环，保持 15 ms 中断短小且有界。
- 环岛问题优先记录 `current_step`、`ring_l`、`ring_r`、关键行边线、`mid_line[80]` 和舵机输出。
- 用连续帧验证状态转移，不用单帧截图代替时序证据。
- 调整控制参数时一次只改一组，保留修改前基线。
- 不手工修改 `project/mdk/out_file/` 中的构建产物。
- 舵机或电机台架测试前，确保车轮离地、限流供电、低输出上限且随时可断电。
- 激光调试时先物理断开或使用安全指示负载，不将激光对准人或反射表面。
