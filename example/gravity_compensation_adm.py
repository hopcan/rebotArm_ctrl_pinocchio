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
adm_scale = [1, 2, 1.5, 1,10,10, 0]
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
            for motor_can_id in list(range(1,8)) :
                print(f"Motor id {handle.motors[motor_can_id].motor_id}")
                print(f"Motor Mit cfg:{handle.motors[motor_can_id].mit_cfg}")
                # print(f"Motor Pos Vel cfg:{handle.motors[motor_can_id].pos_vel_cfg}")
                print(f"Motor Use Modes:{handle.motors[motor_can_id].use_mode}")
                print(f"Motor Pos Max:{handle.motors[motor_can_id].pos_max}")
                print(f"Motor Pos Min:{handle.motors[motor_can_id].pos_min}\n")
            print("Controller is connected and ready.")
        else:
            print("Controller failed to connect.")

        while True:
            joints_pos = handle.return_joints_last_pos()
            motors_last_pos,full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,model,data)  
            for motor_id in list(range(1,8)):
                torque[motor_id-1] = handle.motors[motor_id].state.torq
                tmp = torque[motor_id-1] - full_torques[motor_id-1]
                if motor_id == 7 :
                    tau_ext_processed = smooth_deadzone(tmp, 0.05)
                    gripper_pos = tau_ext_processed *gripper_scale
                    motors_last_pos[6] -= gripper_pos
                else:
                    tau_ext_processed = smooth_deadzone(tmp, 0.1)
                    if abs(tmp) > 0.1 and motor_id == 1 :
                        print(f"error : {tmp}")
                        print(f"compensation :{tau_ext_processed}")
                    full_torques[motor_id-1] = full_torques[motor_id-1] + tau_ext_processed / adm_scale[motor_id-1]
            handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
            time.sleep(0.005)