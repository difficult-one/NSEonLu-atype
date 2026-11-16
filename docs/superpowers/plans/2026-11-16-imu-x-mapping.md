# 远端 IMU X 映射修正计划

目标：将远端轮速输入、融合速度与配对轮速输出的 X 分量改为同号映射，并用四份既有 CSV 验证。

范围：保留 A 板原生姿态、角速度、导航系加速度、116 mm 偏移、滤波参数和观测用途；不改变控制、传输和本地云台路径。CSV 中旧导航系轮速已经包含旧 X 变号，回放必须按对应中间时刻姿态还原后重映射，不能直接重放旧输入作为修正结果。

1. 更新 `tests/chassis_remote_fusion/test_chassis_remote_fusion.c` 的物理夹具，加入恒定导航平移叠加正反自转回归；先运行确认旧映射失败。
2. 修正 `modules/algorithm/chassis_remote_fusion.c` 的输入、输出、配对输出及头文件注释；同步远端 observer 夹具。验证平移、旋转、重复包和掉线回退。
3. 添加 `scripts/replay-imu-x-mapping.py`，按最新连续采集段解码四份原始 CSV。生产 C KF 回放核对旧输出，再仅改变坐标映射比较拒绝数、位移及尾段漂移。独立记录数据哈希、窗口、未知初态和插值限制。
4. 更新当前坐标文档，运行远端/observer/遥测回归、回放工具测试、两板构建与差异检查。独立复核后记录验证结果并创建本地提交。

回放源：`D:/QQ聊天记录/右移动320cm.csv`、`小陀螺行进.csv`、`小陀螺前进320cm.csv`、`小陀螺向左320cm.csv`。原文件只读，完整回放结果保存在 `build/imu-x-mapping-replay/`，仓库仅保存工具与摘要。

以上步骤已完成。最终回放目录为 `build/imu-x-mapping-replay-final/`；结果与限制见 [验证记录](../verification/2026-11-16-imu-x-mapping.md)。
