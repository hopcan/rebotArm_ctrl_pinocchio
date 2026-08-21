# rebotArm_ctrl_pinocchio

Repository with helpers and examples to control the reBot arm and run
Pinocchio-based gravity/kinematics examples.

## 安装
在终端执行：

```bash
conda create -n pinocchio python=3.12
conda activate pinocchio
pip install pyyaml motorbridge
pip install pin
```

## 运行示例
从仓库根目录运行：

```bash
python3 example/gravity_conpensation.py
```



