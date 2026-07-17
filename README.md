# STC32G144K 智能车固件

本工程基于逐飞 STC32G144K 库和 Keil C251 工具链，实现摄像头循迹、左右环岛、白列轮廓视觉避障、编码器测速、电机与舵机闭环、IMU 横摆阻尼、靶点识别、激光时序和 WiFi 调试。DL1B/ToF 代码仅作保留，当前固件不初始化、不轮询。

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
| `project/code/visual_avoidance.c` | 视觉障碍候选确认、左右净空选择和简化避障状态机 |
| `project/code/tof_avoidance.c` | 保留的 ToF 历史实现，当前不加入活跃 Keil 目标 |
| `project/code/assistant_debug.c` | 图像边界、状态叠加、示波器数据和在线调参 |
| `project/code/wifi_assistant.c` | WiFi 连接与调试助手会话 |

## 运行链路

### 图像与转向

```text
摄像头帧完成
  -> Image_OldStyle_Process()
  -> 灰度阈值、参考列与障碍轮廓候选
  -> left_edge_line[] / right_edge_line[]
  -> 候选触发确认、绕行方向与避障状态
  -> Ring() 环岛补线
  -> left_control_line[] / right_control_line[]
  -> mid_line[]
  -> Servo_Loop()
  -> 舵机 PWM
```

`Image_OldStyle_Process()` 在主循环中每帧执行，顺序为：

1. `get_reference_point()` 直接用图像底部 4 行更新黑白和反光阈值；运行时 Otsu 已关闭。
2. `Search_reference_col()` 扫描参考列，同时直接利用已算出的 `globe_remote` 终止行流式检查“缩短—平台—恢复”；不建二值图、不保存整列数组、不二次扫图。
3. `Search_line()` 使用 Git 初版的左右独立搜线规则，并复制到控制线。
4. 视觉避障对原始双跳变候选完成触发行和 3 帧内 2 帧确认；搜线结果只用来选择左/右绕行方向。
5. 避障未抑制环岛时执行 `Ring()`，否则保持环岛为 `NORM`。
6. `Fitted_Midline()` 生成舵机使用的中线。
7. 发车前按过 PB3 时，每 2 帧执行一次 Git 初版风格的单行预扫和纵横粗扫；视觉避障不停止靶点识别。

`Search_line()`、`Ring()` 和 `Fitted_Midline()` 仍逐帧执行，舵机控制链路没有降帧。已删除运行时无消费者的 Otsu、两个 22560 字节图像备份，以及 15 ms 中断中结果无人读取的 102 点中线求和。

### 15 ms 控制任务

按下发车键后才启动 `TIM0_PIT`，回调中仅执行：

```c
Encoder_GetValue();
Gyro_Update();
if (visual_avoid_enabled) {
    VisualAvoid_ControlTick(encoder_data_l, encoder_data_r);
}
Servo_Loop();
Motor_Loop();
Laser_Set_Inhibit(0u);
Laser_Task();
```

图像处理、WiFi 通信和格式化输出不应搬进该中断。发车前 `Gyro_Update()` 随每个图像周期执行，便于电机未启动时检查 Z 轴方向；发车后只由 15 ms 控制任务在 `Servo_Loop()` 前采样。

## 白列轮廓视觉避障

### 检测原理

PB4 在发车前按下后锁存视觉避障。关闭时，`Search_reference_col()` 保留当前的提前结束优化；开启时保证至少扫到第 166 列，并在第 20～166 列、步长 2 的 `globe_remote` 序列中查找深色砖块轮廓：

1. 终止行向下增加至少 12 行，表示白列突然缩短。
2. 中间平台宽 10～60 px，平台内起伏不超过 6 行。
3. 终止行减少至少 12 行，且两侧基线差不超过 8 行。
4. 候选底边位于第 45～105 行，到达第 90 行才允许触发。

`Search_line()` 之后，在 `min(障碍底边 + 6, 110)` 行尝试计算候选框两侧净空，但边线丢失、出画面或框不在两线内缩 8 px 范围时，不再否决障碍触发，而是按障碍中心在第 94 列的左/右位置选择反侧绕行。最近 3 帧中命中 2 帧，且中心和底边变化均不超过 12 px，就确认为障碍。

不增加直道识别。只有环岛状态为 `NORM` 时才累计新障碍确认；原始双跳变候选到达第 90 行后，就先跳过环岛转移并执行 `Ring_Over()`，避免红砖在触发点被环岛状态机抢走。没有原始候选时继续普通循迹，不停车。

### 简化状态机

| 状态 | 行为 |
| --- | --- |
| `FOLLOW` | 原始循迹，接收和确认新候选 |
| `BYPASS` | 选择净空更大一侧加入 ±30 px 路径偏置，速度封顶 165 脉冲/15 ms |
| `RECENTER` | 偏置目标改为 0，保持 10 个控制周期让舵机按 3 px/15 ms 变化率平滑撤销 |

确认障碍后，比较候选框两侧到赛道边线的剩余宽度。右侧净空大于或等于左侧时使用 `-30 px` 目标列偏置，否则使用 `+30 px`；这个符号由当前 `err = mid_line - target_col`的舵机闭环方向决定，净空相等时默认右绕。`BYPASS` 每 15 ms 累加两轮编码器绝对脉冲的平均值，达到 6500 后进入 `RECENTER`。障碍连续消失 2 帧后才重新允许下一次触发。没有故障停车、阶段超时或 ToF 退化分支。

PB3 靶点检测与 PB4 视觉避障可以同时开启。避障全程仍执行原始寻线、靶点识别和激光任务；红砖被误识别为靶点时也不抑制激光。

### 日常只调三个参数

| 参数 | 首版初值 | 含义 |
| --- | ---: | --- |
| `VISUAL_AVOID_TRIGGER_ROW` | 90 | 障碍底边到达该行才允许确认 |
| `VISUAL_AVOID_PATH_BIAS_PX` | 30 px | 右绕使用 `-30 px`，左绕使用 `+30 px` |
| `VISUAL_AVOID_PASS_PULSES` | 6500 | 两轮绝对编码器脉冲平均值的累计门限 |

其余形状阈值作为内部固定值，日常调试不增加参数。这三项是低速起点，不表示已通过实车标定。

### 验证要点

1. 用合成终止行序列覆盖正常路面、单次跳变、平台过窄/过宽、平台抖动、横向黑线和标准双跳变。
2. 用红砖实拍连续帧验证 2/3 帧确认、位置一致性和左右净空选择。
3. 车轮离地确认 `-30` 确实产生右绕转向、`+30` 产生左绕转向，并检查 165 速度封顶、6500 脉冲后平滑回线。
4. 同时锁存 PB3/PB4，检查避障时环岛保持 `NORM`，靶点检测和激光仍正常调度。
5. 完整重建 Keil C251 目标，确认 `0 Error(s), 0 Warning(s)` 并比较 code、const、xdata 与图像帧处理时间。构建通过不代表已完成实车验证。

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
- 左/右环 `IN` 的第 0 行上锚点分别为 `15/172`，保持 `172 = 187 - 15` 的镜像关系；相对上一档 `25/162` 再向外扩大 10 像素，进一步增强环内转向。
- 左/右环 `OUT` 的第 0 行上锚点分别为 `10/177`，保持镜像关系；相对上一档 `20/167` 再向外扩大 10 像素，进一步增强出环转向，同时避免直接使用边缘列 `0/187`。
- `IN/OUT` 中未画斜线的一侧控制线整条贴边：左环将 `left_control_line[]` 置为 `0`，右环将 `right_control_line[]` 置为 `187`；原始 `left_edge_line[]/right_edge_line[]` 扫线不改。
- 左环 `IN/OUT` 读取本侧左下角 `left_edge_line[119]=n`，写入右控制线下锚点 `47 + (47-n)/4`；右环读取本侧右下角 `right_edge_line[119]=n`，写入左控制线下锚点 `141 + (141-n)/4`。负差值按向零取整，最终列坐标仍经过图像宽度限幅。
- 右环 `TURN` 仍保留左控制线底部至少为第 30 列；该固定值不再用于 `IN/OUT`。
- `BACK` 不再搜索局部极值作为上锚点：左环固定连接左线第 40 行，右环固定连接右线第 40 行，下端均为第 119 行。
- 左环 `IN` 从第 20 帧起保留原有的 3 帧边线确认；若边线抖动导致确认始终失败，累计到 35 帧时兜底进入 `OUT`，不会一直卡在 `IN`。
- 右环 `IN` 阶段累计 35 帧后进入 `OUT`（原为 40 帧）。
- `OUT` 阶段持续补线，直到边线满足回正条件。

除上述固定比例的 `IN/OUT` 下锚点映射外，不使用按行宽度学习或上一帧中线退路，避免环内左右控制线跳变。

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
| KEY3 | `IO_PB4` | 视觉避障关闭 | 本次上电启用白列轮廓检测和视觉避障 |

PB3/PB4 按住或短按一次都能在发车前锁存，KEY1 发车后不再将它们作为运行中模式切换。两个功能可同时启用。ToF/DL1B 不因 PB4 而初始化或轮询。

### 编译期开关

| 宏 | 当前值 | 位置 |
| --- | ---: | --- |
| `ZEBRA_DETECT_ENABLE` | 0 | `project/code/image.c` |
| `TARGET_FIND_FRAME_DIV` | 2 | `project/code/image.c` |
| `IMAGE_OTSU_ENABLE` | 0 | `project/code/image2.h` |
| `RIGHT_RING_ENABLE` | 1 | `project/code/ring.c` |
| `ASSISTANT_DEBUG_ENABLE` | 1 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_IMAGE_DIV` | 2 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_SCOPE_ENABLE` | 0 | `project/code/assistant_debug.h` |
| `ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE` | 1 | `project/code/assistant_debug.h` |

`current_step >= 2` 时仍按原环岛规则跳过靶点检测，避免环岛黑区和断线造成误触发。视觉避障会将环岛保持为 `NORM`，因此本身不抑制靶点检测或激光。

### 图像与舵机

| 参数 | 当前值 | 位置 |
| --- | ---: | --- |
| `SEARCH_IMAGE_W` | 188 | `project/code/image.h` |
| `SEARCH_IMAGE_H` | 120 | `project/code/image.h` |
| `PIXEL_OFFSET` | 2 | `project/code/image.h` |
| `STOP_ROW` | 8 | `project/code/image.h` |
| `controlReferenceLine` | 80 | `project/code/servo.h` |
| `Mid_Col` | 94 | `project/code/servo.h` |
| `reference_contrast_ratio` | 32 | `project/code/image.c` |
| `camera_exposure_time` | 110 | `project/code/image.c` |
| `SERVO_DUTY_MIN` | 690 | `project/code/servo.h` |
| `SERVO_DUTY_MID` | 850 | `project/code/servo.h` |
| `SERVO_DUTY_MAX` | 1040 | `project/code/servo.h` |

### 闭环参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `SERVO_KP_TURN_Q10` | 4500 | 转弯舵机 P |
| `SERVO_KP_STRAIGHT_Q10` | 2000 | 直道舵机 P |
| `SERVO_KD_Q10` | 400 | 舵机 D |
| `SERVO_KF_Q10` | 200 | 曲率前馈 |
| `SERVO_KG_Q10` | 1200 | 陀螺仪横摆阻尼 |
| `MOTOR_KP_Q10` | 1024 | 电机 PI 的 P |
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
| `MAX_SPEED` | 700 | 默认最大目标速度，可通过 WiFi 调参 |
| `MAX_SPEED_TUNE_MAX` | 700 | 最大速度的在线调参上限 |
| `MIN_SPEED` | 172 | 大转向默认最低目标速度，可通过 WiFi 调参 |
| `BASE_TARGET_SPEED` | 175 | 闭环启动前的默认目标速度 |
| `WHEEL_TARGET_MIN` | 165 | 差速后的单轮目标速度下限 |
| `TURN_DIFF_MAX` | 8 | 满转角时的单轮差速偏置 |

速度值的单位是 15 ms 窗口内的编码器脉冲数，不是 RPM。目标速度按舵机转角平方降低，中小弯比原线性映射保留更高速度，大转角最低降到 `min_speed`。左右轮再按舵机转向加入最大 8 脉冲的差速偏置，最终单轮目标限制在 165–`max_speed`。WiFi 速度范围通过 `Motor_Set_Speed_Range()` 成组更新，15 ms 控制任务不会读到一半新、一半旧的范围。

`MOTOR_MIN_EFFECTIVE_PWM=2500` 是运行中的硬下限：只要目标速度为正，实际 PWM 不会主动降到 0。若 PWM 2500 对应的机械速度已经高于 WiFi 设定目标，继续降低目标值不会让实车进一步降速。`MOTOR_SAFETY_STOP` 不受该下限约束，会直接把两路 PWM 清零。

## 靶点、激光与 WiFi

- `Pre_Scan()` 与 `Target_find()` 使用 Git 初版风格的单行预扫和纵横粗扫，不再建立约 3000 候选点的底边映射，并更新 `tar_x`、`tar_y` 和 `aim_ready_flag`。
- `Laser_Task()` 在 15 ms 任务中执行，当前发射 2 tick，随后冷却 34 tick。
- WiFi 仅在 `SWITCH2_PIN` 有效时启用。
- 网络名、密码、上位机 IP 和端口位于 `project/code/wifi_assistant.c`，不在文档中复制其实际值。
- WiFi 第 3 通道为带符号 `GYRO_KG`，范围 -3000～3000，默认 1200；原曲率前馈 KF 固定为 200，不再由 WiFi 修改。第 6 通道为最低速度，第 7 通道为最大速度；两者范围为 165–700，并自动维持 `min_speed <= max_speed`，默认分别为 172/700。
- WiFi 通道 1～8 依次为：舵机 KP、舵机 KD、陀螺仪 KG、电机 KP、停机开关、最低速度、最大速度、相机曝光。第 5 通道取 `1` 时普通运行，取 `2` 时立即绕过最低 PWM 与斜率限制、清空两侧 PI 并将两路 PWM 置零；从 `2` 改回 `1` 后恢复普通电机模式。电机 KI 不再在线修改，固定为 `150/Q10`。
- 8 通道示波器当前关闭；若单独开启，通道 4/5 保持显示陀螺仪 Z 轴和校正量，不依赖 ToF 模块。
- WiFi 图像每处理 2 帧发送 1 帧，边线随该图像一起发送；非发送帧不读取避障调试快照、不绘制标注。在线参数接收仍保留，车辆的 15 ms 控制周期和电机目标速度不变。
- 图像左上角使用英文缩写显示环岛状态 `R:NORM/FIRST/ENTER/TURN/IN/OUT/BACK/OVER` 和避障状态 `V:OFF/SCN/RAW/H1/BYP/REC`：`OFF` 表示 PB4 没有锁存，`SCN` 表示已开启且正在扫描，`RAW` 表示当前帧找到原始双跳变，`H1` 表示已有一次确认，`BYP/REC` 分别是绕行/回线。候选有效时同时画出左右界和底边。这些只在实际发送帧上绘制，不做逐帧数字格式化。
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
| DL1B ToF 预留 | SCL P61、SDA P63、XSHUT P66；INT 不接；当前固件不初始化 |
| 发车按键 | `IO_PB2` |
| 靶点检测锁存键 | `IO_PB3`，发车前按下 |
| 视觉避障锁存键 | `IO_PB4`，发车前按下 |
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

2026-07-17 使用同一目标完整 Rebuild 的结果：

```text
Program Size: data=10.6 edata+hdata=8204 xdata=32620 const=5900 code=127968
0 Error(s), 0 Warning(s)
```

相对本次修改前仍编译 ToF/DL1B 的基线（`data=10.4, edata+hdata=8204, xdata=32669, const=14182, code=126289`），视觉版 `xdata -49 B`、`const -8282 B`、`code +1679 B`，`code + const` 合计减少 6603 B。图像帧耗时必须在实车摄像头周期中分别测量 PB4 关闭/开启值，当前桌面构建不冒充硬件计时结果。

ToF/DL1B 源文件保留在磁盘，但不加入当前活跃 Keil 目标；主程序也不初始化、不轮询、不调用其状态机。运行时 Otsu 仍关闭；PB3 开启时，靶点识别每 2 帧执行一次轻量预扫和纵横粗扫；WiFi 开启时每两帧同步发送图像和边线。

构建成功只证明编译与链接通过，不代表环岛、舵机、电机或激光已完成实车验证。

## 调试与安全约束

- 保持图像处理在主循环，保持 15 ms 中断短小且有界。
- 环岛问题优先记录 `current_step`、`ring_l`、`ring_r`、关键行边线、`mid_line[80]` 和舵机输出。
- 用连续帧验证状态转移，不用单帧截图代替时序证据。
- 调整控制参数时一次只改一组，保留修改前基线。
- 不手工修改 `project/mdk/out_file/` 中的构建产物。
- 舵机或电机台架测试前，确保车轮离地、限流供电、低输出上限且随时可断电。
- 激光调试时先物理断开或使用安全指示负载，不将激光对准人或反射表面。
