# STC32G144K 智能车固件

本工程基于逐飞 STC32G144K 库和 Keil C251 工具链，实现摄像头循迹、左右环岛、左边线宽度突变视觉避障、编码器测速、电机与舵机闭环、IMU 横摆阻尼、靶点识别、激光时序和 WiFi 调试。视觉避障默认开启，DL1B ToF 当前不初始化、不轮询。

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
| `project/code/gyro.c` | IMU660RB 自检、Z 轴校零、滤波和舵机横摆修正状态 |
| `project/code/motor.c` | 编码器、目标速度、差速和电机 PWM |
| `project/code/servo.c` | 舵机 PWM 初始化、闭环和安全限幅 |
| `project/code/laser.c` | 靶点检测和激光脉冲/冷却时序 |
| `project/code/visual_avoidance.c` | 左边线宽度双突变的低算力检测和 `PATH_BIAS` 持续避障 |
| `project/code/tof_avoidance.c` | 保留的 ToF 历史实现，当前不加入活跃 Keil 目标 |
| `project/code/assistant_debug.c` | 图像边界、状态叠加、示波器数据和在线调参 |
| `project/code/wifi_assistant.c` | WiFi 连接与调试助手会话 |

## 运行链路

### 图像与转向

```text
摄像头帧完成
  -> Image_OldStyle_Process()
  -> 灰度阈值与参考列
  -> left_edge_line[] / right_edge_line[]
  -> Ring()：FIRST 阶段先判视觉障碍，再顺位判断环岛
  -> left_control_line[] / right_control_line[]
  -> mid_line[]
  -> Servo_Loop()
  -> 舵机 PWM
```

`Image_OldStyle_Process()` 在主循环中每帧执行，顺序为：

1. `get_reference_point()` 用图像底部 4 行、步长 4 的样本更新黑白和反光阈值；运行时 Otsu 已关闭。
2. `Search_reference_col()` 保持第 0～187 列范围，候选列步长为 8；纵向仍按 2 行扫描。底部阈值采样步长为 4。
3. `Search_line()` 使用 Git 初版的左右独立搜线规则，并复制到控制线。
4. `Ring()` 在 `FIRST` 阶段先调用 `VisualAvoid_ProcessFrame()`；仅左环预判 `ring_l` 且处于 `SCAN` 时检测障碍。完整双突变轮廓一成立就立即进入持续避障，直到里程和回正完成都停留在 `FIRST`；未命中才继续原来的左/右环顺位判断。
5. `Fitted_Midline()` 生成舵机使用的中线。
6. 发车前按过 PB3 时，每 2 帧执行一次 Git 初版风格的单行预扫和纵横粗扫；视觉避障不停止靶点识别。

`Search_line()`、`Ring()` 和 `Fitted_Midline()` 仍逐帧执行，舵机控制链路没有降帧。已删除运行时无消费者的 Otsu、两个 22560 字节图像备份，以及 15 ms 中断中结果无人读取的 102 点中线求和。

### 15 ms 控制任务

按下发车键或通过 WiFi 第 5 通道输入 `0` 或 `1` 后启动 `TIM0_PIT`，回调中仅执行：

```c
Encoder_GetValue();
VisualAvoid_ControlTick(encoder_data_l, encoder_data_r);
Gyro_Update();
Servo_Loop();
Motor_Loop();
Laser_Set_Inhibit(0u);
Laser_Task();
```

障碍轮廓检测全部留在图像主循环，只读取已经生成的原始扫线点。15 ms 中断只做两轮编码器均值累加和单字节状态发布，不扫图、不读边线、不调用 `Ring_Over()`。发车前 `Gyro_Update()` 随图像周期执行；发车后只由 15 ms 控制任务在 `Servo_Loop()` 前采样。

## 左边线宽度突变视觉避障

DL1B 不初始化、不轮询，ToF 驱动和旧避障文件只保留在磁盘，不参与当前 Keil 目标。视觉避障默认开启，不额外扫描原图，不改写 `left_control_line[]`、`right_control_line[]` 或 `mid_line[]`，只通过现有 `SERVO_MODE_PATH_BIAS` 改变视觉目标列。

### 检测与控制

`Search_line()` 的原始扫线行为第 119、117、115……9 行，偶数行由 `insert_val()` 后插补。避障只在 `FIRST && ring_l && SCAN` 时运行，`ring_r` 直接跳过。先精确检查原始点 `(109,107)`，然后以 4 行步长粗扫 `107,103,…,27`：

1. 粗扫每次只读一组新的左/右边线并复用上一组；下端左线变化达到 `+8 px` 或上端达到 `-6 px` 时才读取中间原始行。
2. 候选区间被还原为两个 2 行区间：下端宽变窄至少 12 px，上端窄变宽至少 8 px，右线变化不超过 4 px。
3. 双突变成立后才检查粗扫跳过的原始点：中间近竖直左线高 8～40 行、整段横向起伏小于 10 px，两角列差也严格小于 10 px。全程不读偶数插补点。

普通循迹先由环岛预判进入 `FIRST`。完整轮廓一经识别就立即开始一次完整动作：

1. `SCAN → ACTIVE`：设定 `PATH_BIAS=-40 px`，切换到 `SERVO_MODE_PATH_BIAS` 并清零避障里程。偏置每 15 ms 最多变化 6 px。
2. `ACTIVE`：图像侧完全停止障碍扫描，环岛保持 `FIRST`。中断只累加 `(|encoder_l|+|encoder_r|)/2`；累计 6500 脉冲前不会退出避障。
3. `ACTIVE → RECENTER`：达到 6500 后仅把偏置目标改为 0，仍保持 `PATH_BIAS` 模式。
4. `RECENTER → SCAN`：用 8 个控制周期平滑回正；中断只发布完成标志，下一图像帧切回 `SERVO_MODE_VISION`、执行 `Ring_Over()` 并恢复 `NORM`。

手动停车时编码器增量为 0，里程自然暂停，再次发车后继续。避障不会关闭 PB3 靶点检测或激光，不改变 ToF 状态，也不改变电机速度调度和上限。

### 关键参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `VISUAL_AVOID_RIGHT_BIAS_PX` | -40 px | 右绕的视觉目标列偏置 |
| `SERVO_PATH_BIAS_STEP_PER_TICK` | 6 px/15 ms | 偏置加入和撤销的最大变化率 |
| `VISUAL_AVOID_PASS_PULSES` | 6500 | 两轮绝对编码器脉冲均值的累计门限 |
| `VISUAL_AVOID_RECENTER_TICKS` | 8 | 里程到达后的平滑回正周期数 |
| `VISUAL_AVOID_WIDTH_JUMP_PX` | 12 px | 障碍下端宽变窄的最小幅值 |
| `VISUAL_AVOID_UPPER_JUMP_PX` | 8 px | 障碍上端窄变宽的最小幅值 |
| `VISUAL_AVOID_LEFT_POINT_DIFF_PX` | 10 px | 两个左线突变点必须严格小于该列差 |

低速测试时先看 WiFi 状态：轮廓成立后直接显示 `V:AVD`，并在 `ACTIVE/RECENTER` 全程保持。CH6 在整段动作中保留最初的检测行。

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

### 环内补线策略

IN/OUT 阶段使用固定上锚点和按底部原始边线映射出的下锚点补线，避免断边直接进入舵机控制：

- 几何判定中的右侧阈值使用 `RING_MIRROR_COL()` 从 188 像素宽度计算；固定补线上锚点直接填写列坐标。
- `ENTER → TURN` 必须先在近端观测到 `midPoint >= 60`，使 `mid_under_flag` 置位。
- 在 `mid_under_flag` 已置位的前提下，`turn_flag` 成立或 `midPoint < 45` 才允许进入 `TURN`。
- 左/右环 `IN` 的第 0 行上锚点分别为 `15/170`；右环上锚点固定为第 170 列。
- 左/右环 `OUT` 的第 0 行上锚点分别为 `10/170`；右环上锚点同样固定为第 170 列。
- `IN/OUT` 中未画斜线的一侧控制线整条贴边：左环将 `left_control_line[]` 置为 `0`，右环将 `right_control_line[]` 置为 `187`；原始 `left_edge_line[]/right_edge_line[]` 扫线不改。
- 左环 `IN/OUT` 读取 `left_edge_line[119]=n`，下锚点以第 141 列为基准，实际补线列为 `141 + (141-n)/4`；右环读取 `right_edge_line[119]=n`，下锚点以第 10 列为基准，实际补线列为 `10 + (10-n)/4`。负差值按向零取整，最终列坐标仍经过图像宽度限幅。
- 右环 `TURN` 仍保留左控制线底部至少为第 30 列；该固定值不再用于 `IN/OUT`。
- `BACK` 不再搜索局部极值作为上锚点：左环固定连接左线第 40 行，右环固定连接右线第 40 行，下端均为第 119 行。
- 左环 `IN` 从第 20 帧起保留原有的 3 帧边线确认；若边线抖动导致确认始终失败，累计到 35 帧时兜底进入 `OUT`，不会一直卡在 `IN`。
- 右环 `IN` 阶段累计 35 帧后进入 `OUT`（原为 40 帧）。
- `OUT` 阶段持续补线，直到边线满足回正条件。

除上述左环以第 141 列、右环以第 10 列为基准的 `IN/OUT` 下锚点映射外，不使用按行宽度学习或上一帧中线退路，避免环内左右控制线跳变。

### 环岛调试顺序

1. 先确认 `ring_l` / `ring_r` 方向正确。
2. 记录 `current_step` 的连续帧序列，不只看一张截图。
3. 卡在 1：检查首次拐点、宽度和 85/94 行边线。
4. 卡在 2：检查 `midPoint`、`turn_flag`、第 110 行右边线以及 167 阈值。
5. 卡在 3：检查 `rightTopPoint` 和连续 3 帧确认。
6. 卡在 4：检查 50/60 行出口边线。
7. 一次只改一类检测条件、状态转移或补线策略。

## 关键配置

### 发车前功能选择

| 按键 | 引脚 | 默认 | 发车前按下后的锁存行为 |
| --- | --- | --- | --- |
| KEY2 | `IO_PB3` | 靶点检测关闭 | 本次上电启用 `Target_find()` 和打靶 |

PB3 按住或短按一次都能在发车前锁存靶点检测。视觉避障始终启用，PB4 不参与功能选择。

### 编译期开关

| 宏 | 当前值 | 位置 |
| --- | ---: | --- |
| `ZEBRA_DETECT_ENABLE` | 0 | `project/code/image.c` |
| `TARGET_FIND_FRAME_DIV` | 2 | `project/code/image.c` |
| `IMAGE_OTSU_ENABLE` | 0 | `project/code/image2.h` |
| `RIGHT_RING_ENABLE` | 1 | `project/code/ring.c` |
| `ASSISTANT_DEBUG_ENABLE` | 1 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_IMAGE_DIV` | 2 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_SCOPE_ENABLE` | 1 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE` | 1 | `project/code/assistant_debug.h` |

`current_step >= 2` 时仍按原环岛规则跳过靶点检测，避免环岛黑区和断线造成误触发。视觉障碍只在环岛状态 `FIRST` 中判断；命中后本帧不继续环岛判断，但不抑制靶点检测或激光。

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
| `camera_exposure_time` | 110 | `project/code/image.c` |
| `SERVO_DUTY_MIN` | 690 | `project/code/servo.h` |
| `SERVO_DUTY_MID` | 850 | `project/code/servo.h` |
| `SERVO_DUTY_MAX` | 1040 | `project/code/servo.h` |

### 闭环参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `SERVO_KP_TURN_Q10` | 6575 | 转弯舵机 P |
| `SERVO_KP_STRAIGHT_Q10` | 2000 | 直道舵机 P |
| `SERVO_KD_Q10` | 498 | 舵机 D |
| `SERVO_KF_Q10` | 200 | 曲率前馈 |
| `SERVO_KG_Q10` | 22 | 陀螺仪横摆阻尼 |
| `MOTOR_KP_Q10` | 654 | 电机 PI 的 P |
| `MOTOR_KI_Q10` | 150 | 电机 PI 的 I |

PID/PI 参数采用 Q10 定点格式，代码中的整数值等于实际系数乘以 1024。

### 陀螺仪横摆阻尼

- 上电执行 IMU660RB 初始化，然后静止采集 100 个 Z 轴样本求平均零偏，每个样本间隔 5 ms，约需 0.5 秒。
- 不再进行样本跨度判断；只有 IMU660RB 驱动初始化失败时才禁用陀螺仪反馈，此时视觉循线仍可继续工作。
- 校零后的 Z 轴使用 `filtered = (3 * filtered + corrected) / 4` 低通滤波；除以 4 和 Q10 除以 1024 均使用等价移位，减少 15 ms 中断的软件除法。
- 舵机使用 `Kp=4500`、`Kd=400`、`Kf=200`、`KG=1200`，按 `out -= KG * gyro_z / 1024` 加入横摆阻尼；该修正单独限制在 -40～40 duty。
- 首次确认符号时保持车轮离地，手动将车头向右旋转，确认舵机向旋转反方向修正；若方向相反，将 KG 改成负值后再落地测试。

### 电机与速度

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `MOTOR_FREQ` | 17000 | 电机 PWM 频率 |
| `MOTOR_MAX_LIMIT` | 7500 | 最终 PWM 限幅 |
| `MOTOR_MIN_EFFECTIVE_PWM` | 2500 | 正向运行时的最低有效 PWM |
| `MOTOR_PWM_RISE_STEP` | 250 | 每 15 ms 最大 PWM 上升量 |
| `MOTOR_PWM_FALL_STEP` | 400 | 每 15 ms 最大 PWM 下降量 |
| `MAX_SPEED` | 339 | 默认速度调度输入，可通过 WiFi 调参 |
| `MAX_SPEED_TUNE_MAX` | 700 | 最大速度的在线调参上限 |
| `MOTOR_SPEED_GATE_MARGIN` | 80 | 普通运行最终上限相对调度输入减少的脉冲数 |
| `MIN_SPEED` | 183 | 大转向默认最低目标速度，可通过 WiFi 调参 |
| `BASE_TARGET_SPEED` | 175 | 闭环启动前的默认目标速度 |
| `WHEEL_TARGET_MIN` | 165 | 差速后的单轮目标速度下限 |
| `TURN_DIFF_MAX` | 8 | 满转角时的单轮差速偏置 |

速度值的单位是 15 ms 窗口内的编码器脉冲数，不是 RPM。WiFi 输入最大速度 `n` 后，转角平方调度仍按 `n` 到 `min_speed` 计算；普通运行的基础目标和左右单轮目标再统一门控到 `n-80`。例如输入 510 时，调度曲线仍按 510 计算，但两轮实际目标均不超过 430；当 `n-80` 低于 `WHEEL_TARGET_MIN` 时，门控下限保持 165。当 `min_speed` 高于该门控时，最大门控优先，实际目标可以低于设定的 `min_speed`。手动停机继续拥有最高优先级。左右轮按舵机转向加入最大 8 脉冲的差速偏置后仍分别经过该门控。WiFi 速度范围通过 `Motor_Set_Speed_Range()` 成组更新，15 ms 控制任务不会读到一半新、一半旧的范围。

`MOTOR_MIN_EFFECTIVE_PWM=2500` 是运行中的硬下限：只要目标速度为正，实际 PWM 不会主动降到 0。若 PWM 2500 对应的机械速度已经高于 WiFi 设定目标，继续降低目标值不会让实车进一步降速。`MOTOR_SAFETY_STOP` 不受该下限约束，会直接把两路 PWM 清零。

## 靶点、激光与 WiFi

- `Pre_Scan()` 与 `Target_find()` 使用 Git 初版风格的单行预扫和纵横粗扫，不再建立约 3000 候选点的底边映射，并更新 `tar_x`、`tar_y` 和 `aim_ready_flag`。
- `Laser_Task()` 在 15 ms 任务中执行，当前发射 2 tick，随后冷却 34 tick。
- WiFi 仅在 `SWITCH2_PIN` 有效时启用。
- 网络名、密码、上位机 IP 和端口位于 `project/code/wifi_assistant.c`，不在文档中复制其实际值。
- WiFi 第 3 通道为带符号 `GYRO_KG`，范围 -3000～3000，默认 22；原曲率前馈 KF 固定为 200，不再由 WiFi 修改。第 6 通道为最低速度，第 7 通道为调度最大速度 `n`；两者范围为 165–700，并自动维持 `min_speed <= max_speed`，默认分别为 183/339。普通运行最终上限为 `max(165, n-80)`。
- WiFi 通道 1～8 依次为：舵机 KP、舵机 KD、陀螺仪 KG、电机 KP、发车/停车、最低速度、最大速度、相机曝光。第 5 通道输入 `0` 或 `1` 时启动控制定时器并发车，停车后再次输入 `0` 或 `1` 均可恢复；输入 `2` 时立即绕过最低 PWM 与斜率限制、清空两侧 PI 并将两路 PWM 置零。其他输入不会切换发车状态。电机 KI 不再在线修改，固定为 `150/Q10`。
- 8 通道示波器当前开启，每 5 个图像帧发送一次：CH1 中线误差、CH2 舵机误差、CH3/CH4 左右编码器、CH5 陀螺仪 Z 轴、CH6 障碍下突变点行数、CH7/CH8 左右电机 PWM。未检测到障碍时 CH6 为 0；进入 `ACTIVE` 后 CH6 锁存最初触发行，直到回正完成。
- WiFi 图像每处理 2 帧发送 1 帧，边线随该图像一起发送；非发送帧不读取避障调试快照、不绘制标注。在线参数接收仍保留，车辆的 15 ms 控制周期和电机目标速度不变。
- 图像左上角使用英文缩写显示环岛状态 `R:NORM/FIRST/ENTER/TURN/IN/OUT/BACK/OVER` 和视觉避障状态 `V:OFF/SCN/AVD`。`AVD` 表示正处于 `ACTIVE` 或 `RECENTER`；状态只在实际发送图像时绘制。
- 摄像头固定曝光初值为 110；`mt9v03x_init()` 后主程序立即写入该值。WiFi 第 8 通道仍可在运行时修改曝光。

## 硬件连接摘要

| 模块 | 引脚或通道 |
| --- | --- |
| 左电机方向 / PWM | `IO_P52` / `PWMD_CH1_P50` |
| 右电机方向 / PWM | `IO_P53` / `PWMD_CH2_P51` |
| 左编码器 | `PWMC_ENCODER`，读取为 `encoder_data_l` |
| 右编码器 | `PWMA_ENCODER`，读取后取负为 `encoder_data_r` |
| 舵机 PWM | `PWME_CH2P_PA2` |
| IMU660RB | SPI3：SCLK P87、MOSI P85、MISO P86、CS P34 |
| 激光 | `IO_P67` |
| DL1B ToF | 当前不初始化、不轮询；相关引脚未被视觉避障使用 |
| 发车按键 | `IO_PB2` |
| 靶点检测锁存键 | `IO_PB3`，发车前按下 |
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

2026-07-18 使用同一目标完整 Rebuild 的结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32613 const=5845 code=128175
0 Error(s), 0 Warning(s)
```

相对本次重构前的视觉补线版，`code` 增加 1828 字节、`const` 增加 19 字节、`xdata` 增加 46 字节；`visual_avoidance` 模块 ECODE 由 1416 字节增加到 3218 字节。

视觉检测只在 `FIRST && ring_l && SCAN` 时执行；普通无候选帧约读取边线数组 44 次，完整候选帧约 48～63 次。只使用整数加减和比较，不读取偶数插补点、不扫原图、不做浮点运算。进入 `ACTIVE` 后图像侧只做常数级状态检查。

视觉避障已加入当前活跃 Keil 目标；ToF/DL1B 文件仍保留在磁盘但不参与编译。运行时 Otsu 仍关闭；PB3 开启时，靶点识别每 2 帧执行一次轻量预扫和纵横粗扫；WiFi 开启时每两帧发送图像和边线，示波器每 5 帧发送一次。

构建成功只证明编译与链接通过，不代表环岛、舵机、电机或激光已完成实车验证。

## 调试与安全约束

- 保持图像处理在主循环，保持 15 ms 中断短小且有界。
- 环岛问题优先记录 `current_step`、`ring_l`、`ring_r`、关键行边线、`mid_line[80]` 和舵机输出。
- 用连续帧验证状态转移，不用单帧截图代替时序证据。
- 调整控制参数时一次只改一组，保留修改前基线。
- 不手工修改 `project/mdk/out_file/` 中的构建产物。
- 舵机或电机台架测试前，确保车轮离地、限流供电、低输出上限且随时可断电。
- 激光调试时先物理断开或使用安全指示负载，不将激光对准人或反射表面。
