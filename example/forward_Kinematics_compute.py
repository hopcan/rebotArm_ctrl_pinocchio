import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
from motorbridge import Controller

project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))
rebotArm_DM_model_path = project_root/"urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"

from rebotArm_handle import reBotArm_handle
import pinocchio_handle
import time


model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()




if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)

    with reBotArm_handle(ctrl,"rebotDM") as handle:
        if handle.is_connected:
            print("Controller is connected and ready.")
            print("Motor Use Modes:", handle.use_mode)
        else:
            print("Controller failed to connect.")

        while True:
            joints_pos = handle.return_joints_last_pos()
            motors_last_pos,full_torques = pinocchio_handle.gravity_conpensation_control(joints_pos ,model,data)  
            handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)


            Matrix_t ,Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,model,data)
            #compute translation and rotation degree
            x,y,z,roll_deg,pitch_deg,yaw_deg = pinocchio_handle.deal_with_matrix_t_r(Matrix_t,Matrix_r)

            print(f"rotation: roll={roll_deg:.4f}°, pitch={pitch_deg:.4f}°, yaw={yaw_deg:.4f}°")
            print(f"translation: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")
            time.sleep(0.1)