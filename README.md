# STC32G144K 智能车工程说明

这是一个基于逐飞 STC32G144K 库的智能车工程。Keil 工程文件是：

```text
project/mdk/seekfree.uvproj
```

当前真正参与编译的业务代码主要在 `project/code/`，中断文件在 `project/user/isr.c`。`project/user/main.c` 没有加入当前 Keil 工程，只保留作历史文件。

## 先看哪几个文件

如果只是想快速理解工程，建议按这个顺序看：

| 文件 | 主要作用 |
| --- | --- |
| `project/code/main.c` | 程序从这里开始，负责初始化和主循环 |
| `project/code/image.c` | 摄像头图像处理，负责找边线、中线、圆环和目标 |
| `project/code/pid.c` | 舵机和电机的 PID/PI 控制 |
| `project/code/motor.c` | 电机 PWM、编码器读取、目标速度计算 |
| `project/code/servo.c` | 舵机 PWM 输出 |
| `project/code/laser.c` | 识别到目标后控制激光 |
| `project/code/ips_ui.c` | IPS 屏幕调试显示 |

## 程序启动后做了什么

`main.c` 的流程很简单：

1. 初始化系统时钟和调试串口。
2. 初始化电机、编码器、舵机、激光和按键。
3. 初始化摄像头和 IPS 屏幕。
4. 注册一个 15 ms 定时中断。
5. 主循环一直等待摄像头完成一帧图像。

摄像头完成一帧后，主循环会调用：

```c
Image_OldStyle_Process();
```

这个函数会完成一次完整的图像处理。

## 一帧图像怎么处理

图像处理入口在 `project/code/image.c`：

```c
void Image_OldStyle_Process(void)
```

它每次处理一帧摄像头图像，大致顺序是：

1. 复制当前摄像头图像，方便后面调试显示。
2. 用 Otsu 方法估算图像阈值。
3. 根据底部图像亮度更新黑白判断阈值。
4. 找一条比较可靠的参考列。
5. 从下往上寻找左边线和右边线。
6. 调用 `Ring()`，在圆环路段修正控制线。
7. 根据左右控制线生成中线 `mid_line`。
8. 调用 `Pre_Scan()` 和 `Target_find()` 查找目标点。

最后得到的关键结果是：

| 变量 | 含义 |
| --- | --- |
| `left_edge_line[]` | 原始左边线 |
| `right_edge_line[]` | 原始右边线 |
| `left_control_line[]` | 修正后的左控制线 |
| `right_control_line[]` | 修正后的右控制线 |
| `mid_line[]` | 车辆实际用于控制的中线 |
| `tar_flag` | 是否找到目标 |
| `tar_x` / `tar_y` | 目标点坐标 |

## 15 ms 中断做了什么

定时中断回调函数在 `main.c`：

```c
static void Timer0_Callback(void)
```

它每 15 ms 执行一次，只做控制相关的轻量工作：

1. `Encoder_GetValue()` 读取并清零左右编码器。
2. `Servo_Loop()` 根据最新中线计算舵机输出。
3. `Motor_Loop()` 根据目标速度和编码器反馈控制左右电机。
4. 如果 `tar_flag` 为 1，就打开激光；否则关闭激光。

图像处理没有放在中断里，是为了避免中断执行时间太长。

## 常用参数在哪里改

电机相关参数在 `project/code/motor.h`：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `MOTOR_FREQ` | `17000` | 电机 PWM 频率 |
| `MOTOR_MAX_LIMIT` | `7500` | 电机最大 PWM 输出 |
| `MAX_SPEED` | `300` | 直道或小转向时的最高目标速度 |
| `MIN_SPEED` | `180` | 大转向时的最低目标速度 |
| `BASE_TARGET_SPEED` | `300` | 默认目标速度 |

舵机相关参数在 `project/code/servo.h`：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `SERVO_DUTY_MIN` | `690` | 舵机最小限幅 |
| `SERVO_DUTY_MID` | `850` | 舵机中值 |
| `SERVO_DUTY_MAX` | `1040` | 舵机最大限幅 |
| `controlReferenceLine` | `80` | 用哪一行的中线作为主要控制参考 |
| `Mid_Col` | `94` | 188 宽图像的中心列 |

PID 参数在 `project/code/pid.h`。这些参数是 Q10 定点数，也就是实际值放大了 1024 倍保存：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `SERVO_KP_TURN_Q10` | `3500` | 转弯时舵机 P |
| `SERVO_KP_STRAIGHT_Q10` | `2000` | 直道时舵机 P |
| `SERVO_KD_Q10` | `200` | 舵机 D |
| `SERVO_KF_Q10` | `200` | 曲率前馈 |
| `MOTOR_KP_Q10` | `1024` | 电机 PI 的 P |
| `MOTOR_KI_Q10` | `150` | 电机 PI 的 I |

## 当前硬件连接

| 模块 | 引脚或通道 |
| --- | --- |
| 左电机方向 | `IO_P52` |
| 左电机 PWM | `PWMD_CH1_P50` |
| 右电机方向 | `IO_P53` |
| 右电机 PWM | `PWMD_CH2_P51` |
| 左编码器 | `PWMC_ENCODER` |
| 右编码器 | `PWMA_ENCODER`，读取后取反 |
| 舵机 PWM | `PWME_CH2P_PA2` |
| 激光 | `IO_P67` |
| IPS 显示开关 | `SWITCH2_PIN = IO_PB1` |

## IPS 调试显示

按下 `SWITCH2` 时，屏幕会显示当前摄像头画面和调试线：

- 蓝色：左右控制线。
- 红色：中线。
- 黄色：控制参考行。
- 青色：停止搜索行。
- 紫色十字：识别到的目标点。

如果屏幕不显示，先检查 `SWITCH2` 是否按下，以及 `ips200_init()` 是否正常执行。

## 编译和下载

1. 用 Keil MDK for C251 打开 `project/mdk/seekfree.uvproj`。
2. 选择目标 `stc32g144k246`。
3. 点击 Build。
4. 生成文件在 `project/mdk/out_file/`。
5. 将 `project/mdk/out_file/SEEKFREE.hex` 烧录到 STC32G144K。

如果编译时报 `TIM0_PIT` 未定义，就到 `project/code/main.c` 里把下面这一行：

```c
pit_ms_init(TIM0_PIT, CONTROL_PERIOD_MS, Timer0_Callback);
```

改成当前库里实际存在的定时器枚举。

## 修改时注意

- 图像处理放在主循环，不要搬到 15 ms 中断里。
- 电机和舵机输出都有软件限幅，改参数时先小幅调整。
- `libraries/` 是逐飞底层库，一般不要改。
- `project/mdk/out_file/` 是构建产物目录，不建议手动修改里面的文件。
