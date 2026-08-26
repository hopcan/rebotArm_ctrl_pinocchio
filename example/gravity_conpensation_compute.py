import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
from motorbridge import Controller

project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))
rebotArm_DM_model_path = project_root/"urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"


model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()

from rebotArm_handle import reBotArm_handle
import pinocchio_handle
import time
torque = [0]*7
adm_pos = [0]*7
adm_scale = [1.2,5,10, 10,10,10, 10]
if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)

    with reBotArm_handle(ctrl,"rebotDM",config_path = project_root/"config/rebotDM_gravity.yaml") as handle:
        if handle.is_connected:
            print("Controller is connected and ready.")
            print("Motor Use Modes:", handle.use_mode)
        else:
            print("Controller failed to connect.")

        while True:
            joints_pos = handle.return_joints_last_pos()
            motors_last_pos,full_torques = pinocchio_handle.gravity_conpensation_control(joints_pos ,model,data)  
            handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
            time.sleep(0.01)