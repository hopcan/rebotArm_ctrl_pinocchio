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
target_pos = [0.3, 0.0, 0.3 ,0.5 ,0 ,0]


if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)
    positions = [0]*7
    with reBotArm_handle(ctrl,"rebotDM",config_path = project_root/"config/rebotDM_ik.yaml") as handle:
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
        origin_joints_pos = handle.return_joints_last_pos()
        pos = pinocchio_handle.clik_compute_joints_pos(origin_joints_pos ,target_pos, model, data)
        print(pos)
        
        while True:
            if pos != None :
                handle.move_to_joint_positions(positions = pos)

            #compute end point pos
            Matrix_t ,Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,model,data)

            #compute translation and rotation degree
            x,y,z,roll_deg,pitch_deg,yaw_deg = pinocchio_handle.deal_with_matrix_t_r(Matrix_t,Matrix_r)


            print(f"rotation: roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°")
            print(f"translation: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
            time.sleep(0.1)   