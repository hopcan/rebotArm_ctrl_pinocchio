# rebotArm_ctrl_pinocchio

这是一个基于 Pinocchio 的 reBot 机械臂控制示例仓库，包含：

- 机械臂控制封装：`rebotArm_handle.py`
- Pinocchio 运动学/动力学辅助函数：`pinocchio_handle.py`
- 机械臂 URDF 模型与配置文件
- 多个实际运行示例脚本

该项目主要用于：

- 读取并控制 reBot 机械臂电机
- 计算正向运动学与末端位姿
- 计算重力补偿
- 使用闭环逆运动学（CLIK）生成轨迹并执行

## 目录结构

```text
.
├── README.md
├── rebotArm_handle.py
├── pinocchio_handle.py
├── config/
│   ├── rebotDM.yaml
│   ├── rebotDM_ik.yaml
│   └── rebotRS.yaml
├── example/
│   ├── clik_trajectory.py
│   ├── close_loop_inverse_kinematics.py
│   ├── forward_Kinematics_compute.py
│   └── gravity_conpensation_compute.py
├── urdf/
│   ├── 00-arm-rs_asm-v3/
│   └── reBot-DevArm_fixend_description/
└── ...
```

## 环境要求

推荐使用 conda 创建虚拟环境，并安装以下依赖：

```bash
conda create -n pinocchio python=3.12
conda activate pinocchio
pip install pyyaml motorbridge
pip install pin
```

如果你的系统还没有安装 Pinocchio 相关依赖，建议优先按官方安装说明安装对应平台版本。

## 运行前准备

示例脚本中默认使用串口：

```python
channel = "/dev/ttyACM0"
ctrl = Controller.from_dm_serial(channel, 921600)
```

如果你的机械臂连接在其他串口设备上，例如 `/dev/ttyUSB0` 或 `/dev/ttyACM1`，请先修改示例脚本中的串口地址。

另外，项目会加载配置文件和 URDF 模型：

- `config/rebotDM_ik.yaml`
- `config/rebotDM.yaml`
- `urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf`

## 示例脚本说明

### 1. 正向运动学示例

```bash
python example/forward_Kinematics_compute.py
```

用于：

- 读取当前关节角度
- 计算末端位姿
- 输出位姿的平移与欧拉角

### 2. 重力补偿示例

```bash
python example/gravity_conpensation_compute.py
```

用于：

- 读取当前关节状态
- 计算重力矩
- 通过 `move_to_joint_positions(..., torque=...)` 进行补偿控制

### 3. 闭环逆运动学示例

```bash
python example/close_loop_inverse_kinematics.py
```

用于：

- 根据目标末端位姿计算关节解
- 控制机械臂移动到目标点
- 观察末端位置和姿态变化

### 4. 轨迹生成与执行示例

```bash
python example/clik_trajectory.py
```

用于：

- 从当前姿态到目标位姿生成轨迹
- 使用 CLIK 计算一系列关节配置
- 对机械臂按轨迹执行运动

## 常见注意事项

- 该仓库中的示例脚本会直接访问机械臂控制器，必须确保机械臂已连接并处于可操作状态。
- 如果运行脚本时出现 `serial` / `Controller` 相关错误，优先检查：
  - 串口设备路径是否正确
  - 机械臂电源是否开启
  - `motorbridge` 是否已正确安装
- 若出现 Pinocchio 相关错误，检查是否使用了兼容的 Python 版本，以及 `pin` 是否成功安装。

## 相关文件

- `rebotArm_handle.py`：机械臂底层接口与控制逻辑
- `pinocchio_handle.py`：运动学、重力补偿与轨迹生成辅助函数
- `config/*.yaml`：机械臂配置与限制参数
- `urdf/*`：机器人模型描述文件

## 说明

这个项目适合用于学习和调试 reBot 机械臂的控制与 Pinocchio 结合应用，尤其适合进行：

- 关节控制
- 逆运动学求解
- 轨迹生成
- 机器人位姿分析

如果你想要进一步扩展，可以在示例脚本基础上加入：

- 轨迹平滑
- 任务空间约束
- 采样控制与日志记录
- 机械臂状态可视化



