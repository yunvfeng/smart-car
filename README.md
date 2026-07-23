# STC32G144K 智能车固件

本工程基于逐飞 STC32G144K 库和 Keil C251 工具链，实现摄像头循迹、左右环岛、双侧边线宽度突变视觉避障、编码器测速、电机与舵机闭环、IMU 横摆阻尼、靶点识别和激光时序；WiFi 与串口调试代码保留，但当前运行路径均关闭。视觉避障默认开启，DL1B ToF 当前不初始化、不轮询。

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
| 逐飞助手接口 | 当前关闭（`ASSISTANT_DEBUG_ENABLE=0`） |
| WiFi 调试开关 | `SWITCH2_PIN = IO_PB1`（当前无效） |

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
| `project/code/main.c` | 硬件初始化、按键发车和图像主循环 |
| `project/code/image.c` | 阈值、参考列、左右边线、控制中线和目标检测调度 |
| `project/code/ring.c` | 左右环岛检测、状态转移和补线 |
| `project/code/pid.c` | 舵机 PID 和电机 PI |
| `project/code/gyro.c` | IMU660RB 自检、Z 轴校零、滤波和舵机横摆修正状态 |
| `project/code/motor.c` | 编码器、目标速度、差速和电机 PWM |
| `project/code/servo.c` | 舵机 PWM 初始化、闭环和安全限幅 |
| `project/code/laser.c` | 靶点检测和激光脉冲/冷却时序 |
| `project/code/visual_avoidance.c` | 左右边线宽度双突变的低算力检测和 `PATH_BIAS` 持续避障 |
| `project/code/tof_avoidance.c` | 保留的 ToF 历史实现，当前不加入活跃 Keil 目标 |
| `project/code/assistant_debug.c` | 串口逐飞助手、图像边界、状态叠加、示波器数据和在线调参 |
| `project/code/wifi_assistant.c` | 保留的 WiFi 连接实现，当前没有运行时调用 |

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
2. `Search_reference_col()` 保持第 0～187 列范围，候选列步长为 8；纵向仍按 2 行扫描。上一帧参考列只有能贯通到第 10 行才复用，否则重新全范围选列，避免前方边线未经搜索便被填到图像边缘。底部阈值采样步长为 4。
3. `Search_line()` 使用 Git 初版的左右独立搜线规则，并复制到控制线。
4. `Ring()` 在 `FIRST` 阶段先调用 `VisualAvoid_ProcessFrame()`；左环预判检测左侧障碍，右环预判镜像检测右侧障碍。完整双突变轮廓一成立就立即进入持续避障，直到里程和回正完成都停留在 `FIRST`；未命中才继续原来的左/右环顺位判断。
5. `Fitted_Midline()` 生成舵机使用的中线。
6. 发车前按过 PB3 时，每 2 帧运行一次旧版 `Pre_Scan()` / `Target_find()`：先在原图第 100 行预扫目标横向偏移，再沿目标中心纵横扫线确认尺寸和瞄准误差；视觉避障不停止靶点识别。

`Search_line()`、`Ring()` 和 `Fitted_Midline()` 仍逐帧执行，舵机控制链路没有降帧。已删除运行时无消费者的 Otsu、两个 22560 字节图像备份，以及 15 ms 中断中结果无人读取的 102 点中线求和。

### 15 ms 控制任务

按下发车键后启动 `TIM0_PIT`，回调中仅执行：

```c
Encoder_GetValue();
VisualAvoid_ControlTick(encoder_data_l, encoder_data_r);
Gyro_Update();
Servo_Loop();
Motor_Loop();
Laser_Set_Inhibit((!target_detect_enabled || current_step >= 2u) ? 1u : 0u);
Laser_Task();
```

障碍轮廓检测全部留在图像主循环，只读取已经生成的原始扫线点。15 ms 中断只做两轮编码器均值累加和单字节状态发布，不扫图、不读边线、不调用 `Ring_Over()`。发车前 `Gyro_Update()` 随图像周期执行；发车后只由 15 ms 控制任务在 `Servo_Loop()` 前采样。

## 双侧边线宽度突变视觉避障

DL1B 不初始化、不轮询，ToF 驱动和旧避障文件只保留在磁盘，不参与当前 Keil 目标。视觉避障默认开启，不额外扫描原图，不改写 `left_control_line[]`、`right_control_line[]` 或 `mid_line[]`，只通过现有 `SERVO_MODE_PATH_BIAS` 改变视觉目标列。

### 检测与控制

`Search_line()` 的原始扫线行为第 119、117、115……9 行，偶数行由 `insert_val()` 后插补。避障只在 `FIRST && SCAN` 时运行：`ring_l` 直接检测左边线，`ring_r` 将右边线镜像到同一坐标系复用完全相同的检测。先精确检查原始点 `(109,107)`，然后以 4 行步长粗扫 `107,103,…,19`：

1. 粗扫每次只读一组新的左右边线并复用上一组；障碍侧下端变化达到 `+8 px` 或上端达到 `-4 px` 时才读取中间原始行。
2. 候选区间被还原为两个 2 行区间：下端宽变窄至少 9 px，上端窄变宽至少 6 px，另一侧边线变化不超过 4 px。
3. 双突变成立后才检查粗扫跳过的原始点：障碍侧近竖直线高 6～40 行、整段横向起伏小于 10 px，两角列差也严格小于 10 px。全程不读偶数插补点。

普通循迹先由环岛预判进入 `FIRST`。完整轮廓一经识别就立即开始一次完整动作：

1. `SCAN → ACTIVE`：左侧障碍设定 `PATH_BIAS=-40 px` 向右绕，右侧障碍设定 `PATH_BIAS=+40 px` 向左绕；随后切换到 `SERVO_MODE_PATH_BIAS` 并清零避障里程。偏置每 15 ms 最多变化 6 px。
2. `ACTIVE`：图像侧完全停止障碍扫描，环岛保持 `FIRST`。中断只累加 `(|encoder_l|+|encoder_r|)/2`；累计 6000 脉冲前不会退出避障。
3. `ACTIVE → RECENTER`：达到 6000 后仅把偏置目标改为 0，仍保持 `PATH_BIAS` 模式。
4. `RECENTER → SCAN`：用 8 个控制周期平滑回正；中断只发布完成标志，下一图像帧切回 `SERVO_MODE_VISION`、执行 `Ring_Over()` 并恢复 `NORM`。

手动停车时编码器增量为 0，里程自然暂停，再次发车后继续。避障不会关闭 PB3 靶点检测或激光，不改变 ToF 状态，也不改变电机速度调度和上限。

### 关键参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `VISUAL_AVOID_RIGHT_BIAS_PX` | -40 px | 右绕的视觉目标列偏置 |
| `VISUAL_AVOID_LEFT_BIAS_PX` | +40 px | 左绕的视觉目标列偏置 |
| `SERVO_PATH_BIAS_STEP_PER_TICK` | 6 px/15 ms | 偏置加入和撤销的最大变化率 |
| `VISUAL_AVOID_PASS_PULSES` | 6000 | 两轮绝对编码器脉冲均值的累计门限 |
| `VISUAL_AVOID_RECENTER_TICKS` | 8 | 里程到达后的平滑回正周期数 |
| `VISUAL_AVOID_WIDTH_JUMP_PX` | 9 px | 障碍下端宽变窄的最小幅值 |
| `VISUAL_AVOID_UPPER_JUMP_PX` | 6 px | 障碍上端窄变宽的最小幅值 |
| `VISUAL_AVOID_LEFT_POINT_DIFF_PX` | 10 px | 两个左线突变点必须严格小于该列差 |

如后续重新启用逐飞助手，低速测试时可观察状态：轮廓成立后直接显示 `V:AVD`，并在 `ACTIVE/RECENTER` 全程保持。CH6 在整段动作中保留最初的检测行。

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
- `FIRST → ENTER` 保留拐点判定和稳定的第 94/85 行双贴边条件；上方宽度门控在第 25～65 行每隔 5 行检查一次，任一行赛道宽度超过 110 px 即通过。左环保持原贴边条件，右环允许边线落在距右边界不超过 4 px 的范围内，并使用 4 帧 FIRST 丢失确认窗口。
- `ENTER → TURN` 必须先在近端观测到 `midPoint >= 60`，使 `mid_under_flag` 置位。
- 在 `mid_under_flag` 已置位的前提下，`turn_flag` 成立或 `midPoint < 45` 连续满足 2 帧才进入 `TURN`。
- 左/右环 `IN` 的第 0 行上锚点分别为 `15/175`；右环上锚点固定为第 175 列。
- 左/右环 `OUT` 的第 0 行上锚点分别为 `10/175`；右环上锚点同样固定为第 175 列。
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
4. 卡在 2：检查 `midPoint`、`turn_flag`、第 110 行右边线以及右环折中的 169 列阈值。
5. 卡在 3：检查 `rightTopPoint` 和连续 3 帧确认。
6. 卡在 4：检查 50/60 行出口边线。
7. 一次只改一类检测条件、状态转移或补线策略。

## 关键配置

### 发车前功能选择

| 按键 | 引脚 | 默认 | 发车前按下后的锁存行为 |
| --- | --- | --- | --- |
| KEY2 | `IO_PB3` | 靶点检测关闭 | 本次上电启用旧版原图预扫和打靶 |

PB3 按住或短按一次都能在发车前锁存靶点检测。视觉避障始终启用，PB4 不参与功能选择。

### 编译期开关

| 宏 | 当前值 | 位置 |
| --- | ---: | --- |
| `ZEBRA_DETECT_ENABLE` | 0 | `project/code/image.c` |
| `TARGET_FIND_FRAME_DIV` | 2 | `project/code/image.c` |
| `Pre_Line` | 100 | `project/code/laser.c` |
| `AimTolerance` | 5 px | `project/code/laser.c` |
| `IMAGE_OTSU_ENABLE` | 0 | `project/code/image2.h` |
| `RIGHT_RING_ENABLE` | 1 | `project/code/ring.c` |
| `ASSISTANT_DEBUG_ENABLE` | 0 | `project/code/assistant_debug.h` |
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
| `camera_exposure_time` | 512 | `project/code/image.c` |
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

普通循迹采用连续双 KP：中线误差绝对值不超过 8 px 时使用直道 KP 2000，超过 8 px 后连续过渡到可在线调节的转弯 KP，避免增益突跳。普通直道（误差不超过 10 px 且 80/100 行中线差不超过 6 px）的舵机输出每 15 ms 最多变化 8 duty；普通转弯最多变化 24 duty。误差超过 20 px 且舵机继续向同侧加深转向时提升到 40 duty 快速救车；目标向中位回正或跨越中位换向时优先限制为 18 duty，避免连续 S 弯高速反打。环岛和 `PATH_BIAS/CENTER` 不受该变化率限制。

### 陀螺仪横摆阻尼

- 上电执行 IMU660RB 初始化，然后静止采集 100 个 Z 轴样本求平均零偏，每个样本间隔 5 ms，约需 0.5 秒。
- 不再进行样本跨度判断；只有 IMU660RB 驱动初始化失败时才禁用陀螺仪反馈，此时视觉循线仍可继续工作。
- 校零后的 Z 轴使用 `filtered = (3 * filtered + corrected) / 4` 低通滤波；除以 4 和 Q10 除以 1024 均使用等价移位，减少 15 ms 中断的软件除法。
- 舵机使用转弯 `Kp=6575`、直道 `Kp=2000`、`Kd=498`、`Kf=200`、`KG=22`，按 `out -= KG * gyro_z / 1024` 加入横摆阻尼；该修正单独限制在 -40～40 duty。
- 首次确认符号时保持车轮离地，手动将车头向右旋转，确认舵机向旋转反方向修正；若方向相反，将 KG 改成负值后再落地测试。

### 电机与速度

PB2 发车并启动 15 ms 控制定时器后，双轮使用速度调度、转向差速和左右增量式 PI 闭环。上电零输出、编码器 15 ms 采样与 `MOTOR_SAFETY_STOP` 立即清零保持不变。

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `MOTOR_FREQ` | 17000 | 电机 PWM 频率 |
| `MOTOR_MAX_LIMIT` | 7500 | 最终 PWM 限幅 |
| `MOTOR_MIN_EFFECTIVE_PWM` | 2250 | 闭环模式下的正向最低有效 PWM |
| `MOTOR_PWM_RISE_STEP` | 250 | 每 15 ms 最大 PWM 上升量 |
| `MOTOR_PWM_FALL_STEP` | 400 | 每 15 ms 最大 PWM 下降量 |
| `MAX_SPEED` | 280 | 默认速度调度输入；当前普通运行实际上限为 250 |
| `MAX_SPEED_TUNE_MAX` | 700 | 最大速度的在线调参上限 |
| `MOTOR_SPEED_GATE_MARGIN` | 30 | 普通运行最终上限相对调度输入减少的脉冲数 |
| `MIN_SPEED` | 200 | 大转向默认最低目标速度 |
| `BASE_TARGET_SPEED` | 200 | 闭环启动前的默认目标速度 |
| `WHEEL_TARGET_MIN` | 200 | 差速后的单轮目标速度下限 |
| `TURN_DIFF_MAX` | 8 | 满转角时的单轮差速偏置 |

速度值的单位是 15 ms 窗口内的编码器脉冲数，不是 RPM。当前转角平方调度按 280 到 200 计算，基础目标和左右单轮目标统一门控到 `280-30=250`，所以正常运行实际目标范围为 200～250。手动停机继续拥有最高优先级。

闭环模式下，`MOTOR_MIN_EFFECTIVE_PWM=2250` 是运行中的硬下限：只要目标速度为正，实际 PWM 不会主动降到 0。`MOTOR_SAFETY_STOP` 会立即把两路 PWM 清零。

## 靶点、激光与调试通信

- `Pre_Scan()` 每 2 帧在原图第 100 行内以 2 列步长寻找黑色目标段，估算其相对图像中心的横向偏移；`Target_find()` 再沿该中心纵向、横向扫线，目标宽高在 10～44 px 且瞄准误差小于 5 px 时允许发射。
- `Laser_Task()` 在 15 ms 任务中输出 2 tick（30 ms）脉冲，随后冷却 34 tick。P61、P63、P66 是同一发射输出，发射时三路同时为高；当前版本没有蜂鸣器联动。
- `ASSISTANT_DEBUG_ENABLE=0`：当前不初始化、不接收也不发送逐飞助手数据，不接受串口远程发车或在线参数。
- 保留的传输配置为 `ASSISTANT_DEBUG_DEVICE=SEEKFREE_ASSISTANT_DEBUG_UART`；后续重新启用时，当前逐飞库会映射到核心板 USB CDC 虚拟串口。
- WiFi 不初始化、不轮询、不发送，也不参与远程控制；PB1/SWITCH2 当前不产生 WiFi 行为。
- 串口逐飞助手通道 1～8 依次为舵机 KP、舵机 KD、陀螺仪 KG、电机 KP、发车/停车、最低速度、最高调度速度和相机曝光；速度在线调参下限为 200。
- 8 通道示波器每 5 个图像帧发送一次：CH1 中线误差、CH2 舵机误差、CH3/CH4 左右编码器、CH5 陀螺仪 Z 轴、CH6 障碍下突变点行数、CH7/CH8 左右电机 PWM。
- 图像每处理 2 帧发送 1 帧，边线随图像一起发送；发送图像左上角显示环岛和视觉避障状态。
- 摄像头固定曝光为 120；当前串口调试关闭，运行时不会由逐飞助手修改。

## 硬件连接摘要

| 模块 | 引脚或通道 |
| --- | --- |
| 左电机方向 / PWM | `IO_P74` / `PWMB_CH3_P76` |
| 右电机方向 / PWM | `IO_P75` / `PWMB_CH4_P77` |
| 左编码器 | `PWMC_ENCODER`，读取为 `encoder_data_l` |
| 右编码器 | `PWMA_ENCODER`，读取后取负为 `encoder_data_r` |
| 舵机 PWM | `PWME_CH2P_PA2` |
| IMU660RB | SPI3：SCLK P87、MOSI P85、MISO P86、CS P34 |
| 激光 | P61、P63、P66（三路同时输出） |
| DL1B ToF | 当前不初始化、不轮询；相关引脚未被视觉避障使用 |
| 发车按键 | `IO_PB2` |
| 靶点检测锁存键 | `IO_PB3`，发车前按下 |
| 逐飞助手 | 当前关闭；保留核心板 USB CDC 配置 |
| WiFi 调试开关 | `IO_PB1`（当前无效） |

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

2026-07-23 完整回退到凹陷打靶改造之前、保留曝光 50 的历史基线后，完整 Rebuild 结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32650 const=5864 code=130041
0 Error(s), 0 Warning(s)
```

该尺寸与 2026-07-22 11:32 的打靶前 `exposure50` 构建记录完全一致。

将当前固定曝光改为 512 后的完整 Rebuild 结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32650 const=5864 code=130041
0 Error(s), 0 Warning(s)
```

速度改为 min=150、max=250、实际上限=200、最低有效 PWM=1500，并关闭 WiFi 运行路径后的完整 Rebuild 结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32647 const=5864 code=129834
0 Error(s), 0 Warning(s)
```

保持上述速度参数，将逐飞助手传输层改为 USB CDC 虚拟串口后的完整 Rebuild 结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32647 const=5864 code=129922
0 Error(s), 0 Warning(s)
```

相对本次重构前的视觉补线版，`code` 增加 1828 字节、`const` 增加 19 字节、`xdata` 增加 46 字节；`visual_avoidance` 模块 ECODE 由 1416 字节增加到 3218 字节。

视觉检测只在 `FIRST && ring_l && SCAN` 时执行；普通无候选帧约读取边线数组 44 次，完整候选帧约 48～63 次。只使用整数加减和比较，不读取偶数插补点、不扫原图、不做浮点运算。进入 `ACTIVE` 后图像侧只做常数级状态检查。

视觉避障已加入当前活跃 Keil 目标；ToF/DL1B 文件仍保留在磁盘但不参与编译。运行时 Otsu、WiFi 和串口逐飞助手均关闭；PB3 开启时，旧版靶点识别每 2 帧扫描一次原图。

构建成功只证明编译与链接通过，不代表环岛、舵机、电机或激光已完成实车验证。

## 调试与安全约束

- 保持图像处理在主循环，保持 15 ms 中断短小且有界。
- 环岛问题优先记录 `current_step`、`ring_l`、`ring_r`、关键行边线、`mid_line[80]` 和舵机输出。
- 用连续帧验证状态转移，不用单帧截图代替时序证据。
- 调整控制参数时一次只改一组，保留修改前基线。
- 不手工修改 `project/mdk/out_file/` 中的构建产物。
- 舵机或电机台架测试前，确保车轮离地、限流供电、低输出上限且随时可断电。
- 激光调试时先物理断开或使用安全指示负载，不将激光对准人或反射表面。
