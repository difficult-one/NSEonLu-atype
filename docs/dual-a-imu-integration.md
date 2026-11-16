# 双 A 板 IMU 采集与 116 mm 安装补偿

整车主板继续控制底盘、云台等设备；下方 A 板只采集 MPU6500、运行 INS/温控并发送 CAN 数据。两板都使用 `ONE_BOARD` 编译角色，采集板由 `IMU_CAPTURE_ONLY` 限制任务，并非 `CHASSIS_BOARD/GIMBAL_BOARD` 的控制分板模式。旧 DM 模块接收、协议换算和接收端重力扣除已移除。

## 安装坐标

远端融合直接采用 **A 板原生 IMU 坐标**，将原点平移到底盘中心，三根轴保持与 A 板原生轴平行。当前照片中 USB 朝上、XT30 朝下、板面向上，轴向如下：

| A 板原生轴 | 照片方向 |
| --- | --- |
| +X | 向左 |
| +Y | 向下 |
| +Z | 离开板面、朝上 |

用户已确认 116 mm 量到 MPU6500 芯片位置，不是 A 板中心。底盘中心到芯片沿 A 板 +Y，因此 [远端配置](../config/30_velocity_observation/chassis_remote_imu_config.h) 写为：

```c
#define CHASSIS_REMOTE_IMU_OFFSET_BODY_M {0.0f, 0.116f, 0.0f}
#define CHASSIS_REMOTE_IMU_SENSOR_TO_BODY {1.0f, 0.0f, 0.0f, 0.0f}
```

机体系与原生传感器轴同向，安装四元数使用恒等旋转 `[1,0,0,0]`。源四元数仍表示原生传感器到导航系的实时姿态，角速度仍在原生传感器系；源 INS 的去重力加速度位于导航系，不能直接当成 A 板机体系分量。主板在融合前扣除旋转杆臂的二阶导数，包含向心和切向项；不修改原始 IMU 遥测、不重复扣除重力。`CHASSIS_REMOTE_IMU_CONFIG_VERIFIED=0` 保留尚未完成实测验收的状态。

轴向依据来自 [RoboMaster 官方 A 板原理图与位号图](https://cdn-hz.robomaster.com/tem/RoboMaster%20%E5%BC%80%E5%8F%91%E6%9D%BF%E5%8E%9F%E7%90%86%E5%9B%BE%26%E4%BD%8D%E5%8F%B7%E5%9B%BE.zip) 和 [TDK MPU-6500 数据手册第 36 页图 14](https://product.tdk.cn/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6500-datasheet2.pdf)：官方板图中 J10（XT30）在左、J14（USB）在右，U3 的 1 脚位于芯片左下角；芯片 1 脚角对应 `(-X,+Y)`，+Z 离开封装上表面。因此官方板图中原生 +X 向上、+Y 向左，转到当前照片方向后得到上表。

现有轮速、控制与 VOFA 速度公共接口保留原有定义。根据 2026-11-16 四份实测 CSV 的坐标一致性分析，公共轮速 `(vx,vy)` 进入远端融合时按 A 板分量 `(+vx,+vy)` 处理；导航系结果转回 A 板后，以公共 `(+vA_x,+vA_y)` 输出。配对轮速输出同样保持 X/Y 同号，失效时直接回退公共轮速。此前入口、出口各取反 X 的约定会使自转行进的导航速度方向错误旋转，已同步修正；不改变原生 IMU 数据和 +Y 116 mm 配置。照片中的原生轴向不能单独证明公共 `vx` 与实物车头的对应关系，仍需实车正反向检查。

## 接线与构建

两板通过所选 CAN 总线连接（默认 CAN1）：CAN_H 对 CAN_H、CAN_L 对 CAN_L、参考地相连，使用收发器后的总线接口。当前速率 1 Mbps，标准 ID `0x180`；按整条总线两端设置终端电阻。主板 UART7 接 VOFA+ 接收器，下方板无需单独串口记录。

IMU ID 由两板共用的 [`CHASSIS_IMU_CAN_DATA_ID`](../modules/imu/chassis_imu_can_protocol.h) 定义。`0x180` 的仲裁优先级高于 DJI 电机命令和反馈，会让同时等待的电机报文稍后发送；它不减少总线流量，也不改变 1 ms 发送邮箱等待、10 ms 组包期限或电调反馈频率。此前使用 `0x520` 的固件与当前版本不能混用，修改 ID 后需要重新构建并烧录两板，再检查 IMU 更新及电机控制、反馈是否正常。

在 [`config/90_board/board_config.h`](../config/90_board/board_config.h) 中修改 `CHASSIS_IMU_CAN_PORT`：`1`=CAN1（默认），`2`=CAN2，其他值编译报错。`BOARD_CHASSIS_IMU_CAN` 由该宏派生，无需单独修改。两板的默认构建预设共用此头文件；切换后分别重新构建、烧录两板，并将两端接到对应的 CAN1 或 CAN2 接口。该宏只选择这条 IMU 链路，电机等设备保留各自绑定。

手动 Make 构建可附加 `CHASSIS_IMU_CAN_PORT=1` 或 `2`，CMake 使用 `-DCHASSIS_IMU_CAN_PORT=1` 或 `2`，并保留各板原有角色与采集开关。显式构建参数优先于头文件默认值；已有 CMake 缓存若设置过该参数，用 `-UCHASSIS_IMU_CAN_PORT` 重新配置可恢复读取头文件默认值。两板使用相同选值，并核对实际接线。

[板卡预设](../config/00_flash/board_flash_config.h) 的目标 `1` 为整车主板，目标 `2` 为下方采集板，各用独立目录：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Show -Board 1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Build -Board 1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Show -Board 2
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Build -Board 2
# 下载器连接对应物理板后，分别执行对应目标的烧录命令
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Flash -Board 1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build-board.ps1 -Action Flash -Board 2
```

主板开启接收和远端融合；采集板开启 `IMU_CAPTURE_ONLY=1 IMU_CAPTURE_CAN_TX_ENABLE=1`，关闭整车观测、UART/USB 遥测和电机任务。脚本不会识别实际接入的板卡，构建通过也不代表已经烧录。

## 数据与时间

采集任务名义每 20 ms 尝试发送一组，由 9 个 8 字节 CAN 分片组成，携带四元数、去重力加速度、角速度、源样本序号、源毫秒时间、发送时源年龄和有效标志。主板校验顺序、tag、CRC16/CCITT-FALSE 和 10 ms 期限；缺片、错序或过期不刷新已发布数据。

两板时钟独立。主板用“首片接收时间 − 发送时源年龄”估计轮速配对时刻，中心补偿和滤波 dt 使用源板样本间隔。源年龄应 ≤20 ms，已知年龄下界应 ≤60 ms；首片仲裁等待和传感器内部延迟仍未测量。断流后四轮有效则回退轮速，恢复后重新预热；融合只用于观测。

日常 `config/_vofa/vofa_config.h` 保持 `VOFA_OBSERVATION_PROFILE=2`：主板 UART7、JustFloat、230400 / 8N1、20 ms 目标周期，schema 5 的 96 通道，`imu_source=1` 表示第二块 A 板。保存完整 `I0`–`I95` 与原表头。旧 profile 0/schema 3 保留 `I64=2` 的外部来源编码，bit 19 不再设置；两个 schema 的列号、来源编码和单位不同。

## 轴向与补偿检查

在底盘零输出、板固定的条件下，先静止等待 INS 初始化。用主板卡尔曼档 `imu_gx_radps/imu_gy_radps/imu_gz_radps` 检查绕 A 板 +X、+Y、+Z 的右手正转：对应原生陀螺仪轴应为正值，反转为负值。照片方向按上表核对，不给原始角速度再套用公共速度的前/右定义。

再做水平及倾斜后静止检查：源导航系去重力加速度应回到接近零，导航系分量不能直接当作 A 板机体系分量。沿 A 板正反 X/Y 分别平移、起停，检查公共接口中 `vx=+vA_x`、`vy=+vA_y`，并确认轮速、融合结果与配对轮速一致；远端断流回退后符号应保持。绕中心正反旋转并起停，比较原始 IMU 与融合输入中心加速度，检查 +Y 116 mm 杆臂补偿。

缺少独立运动参考时，不将观察到 FUSED 当作精度验收；具体试次见 [实测手册](kalman-chassis-test-and-acceptance.md)。四份 CSV 的 X 映射修正、回放命令与剩余误差见 [2026-11-16 验证记录](superpowers/verification/2026-11-16-imu-x-mapping.md)。
