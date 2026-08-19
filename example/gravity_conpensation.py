import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
from motorbridge import Controller

project_root = Path(__file__).resolve().parent.parent
print(project_root)
sys.path.append(str(project_root))
rebotArm_DM_model_path = project_root/"urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"


model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()

from rebotArm_handle import reBotArm_handle
import time

def gravity_conpensation_control(handle) :
    motors_last_pos=[0]*7
    motor_last_state = handle.motor_state
    for motor_id in list(range(1, 8)) :
        motors_last_pos[motor_id - 1] =  motor_last_state[motor_id].pos
    pin.computeGeneralizedGravity(model, data,  np.array(motors_last_pos[:6]))
    torque = data.g.copy()
    full_torques = torque.tolist() + [0.0] 
    full_torques[3] *= 2.5    
    handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
    return full_torques
            
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
            gravity_conpensation_control(handle)
            time.sleep(0.1)