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
adm_scale = [1, 3, 1 , 10,10,10, 0]
gripper_pos = 0
gripper_scale = 2.5

def smooth_deadzone(x, threshold=0.02):
    if abs(x) < threshold:
        return 0.0
    else:
        # 指数平滑过渡，避免突变
        return np.sign(x) * (abs(x) - threshold)



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
            # print(f"gravity torques : {full_torques}\n")
            for motor_id in list(range(1,8)):
                torque[motor_id-1] = handle.motor_state[motor_id].torq
                tmp = torque[motor_id-1] - full_torques[motor_id-1]
                if motor_id == 7 :
                    tau_ext_processed = smooth_deadzone(tmp, 0.05)
                    gripper_pos = tau_ext_processed *gripper_scale
                    motors_last_pos[6] -= gripper_pos
                    # print(f"gripper pos : {motors_last_pos[6]}")
                    # if gripper_pos != 0.0 :
                    #     print(f"gripper error : {gripper_pos}")
                else:
                    tau_ext_processed = smooth_deadzone(tmp, 0.1)
                    
                    full_torques[motor_id-1] = full_torques[motor_id-1] - tau_ext_processed / adm_scale[motor_id-1]
            handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
            # print(f"real torques : {torque}\n")
            # print(f"output torques : {full_torques}\n")
            # print(f"adm_pos : {adm_pos}\n")
            time.sleep(0.005)