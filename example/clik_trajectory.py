import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
import time
from motorbridge import Controller

project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))
rebotArm_DM_model_path = project_root/"urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"

from rebotArm_handle import reBotArm_handle
import pinocchio_handle
import time

model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()

q_joints = [0]*7
if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)

    with reBotArm_handle(ctrl,"rebotDM",config_path = project_root/"config/rebotDM_ik.yaml") as handle:
        if handle.is_connected:
            print("Controller is connected and ready.")
            print("Motor Use Modes:", handle.use_mode)
        else:
            print("Controller failed to connect.")

        # orginal_joints_pos = [0,0,0,0,0,0]
        target_pos_point = [0.34 ,0 ,0 ,-0.3 ,1.04 ,-0.37]# 0.24 0 0.03 0 0.67 0   0.34 0 0 -0.3 1.04 -0.37  -0.03 0.02 0.41 -0.01 -0.06 0.11 
        Time = 3
        dt = 0.3
        traj_finish_flag = False
        joints_pos = handle.return_joints_last_pos()
        orginal_joints_pos = joints_pos[:6]
        q_traj, q_num = pinocchio_handle.generate_traj(original_joints_pos = orginal_joints_pos,target_pos_point = target_pos_point,T = Time,dt = dt,model = model, data = data,frame_id = None)
        
        while True:
            while traj_finish_flag == False:
                for Num in range(0,q_num) :
                    q_joints[:6] = q_traj[Num]
                    if Num < q_num:
                        handle.move_to_joint_positions(positions = q_joints)
                        time.sleep(0.1)
                traj_finish_flag = True
                print("轨迹执行完毕！")  


