import pinocchio as pin
import numpy as np
import yaml
from pathlib import Path

project_root = Path(__file__).resolve().parent


def gravity_conpensation_control(joints_pos,model,data) :
    motors_last_pos = joints_pos
    full_torques = np.zeros(7)

    #compute gravity
    pin.computeGeneralizedGravity(model, data,  np.array(motors_last_pos[:6]))
    torque = data.g
    full_torques[:6] = torque 

    #add conpensation
    full_torques[3] *= 2.7   

    return motors_last_pos,full_torques.tolist()




def forwardKinematics_compute (handle,model,data,joint_nums = None) :
    if joint_nums == None :
        joint_nums = 7
    frame_name = ["base_link","link1","link2","link3","link4","link5","link6","end_link"]
    motors_now_pos = [0]*7
    frame_id = model.getFrameId(frame_name[joint_nums])

    #get joints pos
    for motor_id in list(range(1, 8)) :
        motors_now_pos[motor_id - 1] =  handle.motor_state[motor_id].pos
    pin.forwardKinematics(model, data, np.array(motors_now_pos[:6])) 
    pin.pin.updateFramePlacements(model, data)

    return data.oMf[frame_id].translation , data.oMf[frame_id].rotation



def deal_with_matrix_t_r(Matrix_t,Matrix_r):
    #compute translation and rotation degree
    x,y,z = Matrix_t
    rpy = pin.rpy.matrixToRpy(Matrix_r)

    roll_deg = rpy[0] * 180 / 3.14159
    pitch_deg = rpy[1] * 180 / 3.14159
    yaw_deg = rpy[2] * 180 / 3.14159
    return x,y,z,roll_deg,pitch_deg,yaw_deg




def clik_compute_joints_pos(origin_joints_pos,target_point,model,data,joint_nums = None):
    damp = 1e-12
    DT = 1e-1
    positions = [0]*7
    iteration_nums = 0
    Max_iteration_nums = 1000
    frame_name = ["base_link","link1","link2","link3","link4","link5","link6","end_link"]
    #default gripper_end 
    if joint_nums == None :
        joint_nums = 7  
    
    frame_id = model.getFrameId(frame_name[joint_nums])
    rot = pin.rpy.rpyToMatrix(target_point[3], target_point[4], target_point[5])
    oMdes = pin.SE3(rot, np.array(target_point[:3]))
    q = np.array(origin_joints_pos[:6])
    positions[6] = origin_joints_pos[6]
    while True:
        pin.forwardKinematics(model, data, q)
        pin.pin.updateFramePlacements(model, data)
        iMd = data.oMf[frame_id].actInv(oMdes) # actInv 计算自身的逆并立即作用于目标矩阵
        #compute the err between target and frame 
        err = pin.log(iMd).vector 
        if np.linalg.norm(err) <= 0.01 :   #默认计算 L2 范数，位置和姿态总偏差半径小于 0.01
            success = True
            break
        if iteration_nums >= Max_iteration_nums :
            success = False
            break
        J = pin.computeFrameJacobian(model, data, q, frame_id)
        manipulability = np.sqrt(np.linalg.det(J[:6,:] @ J[:6,:].T))
        if manipulability < 0.001:
            print("接近奇异点")  
            return None
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)  #Jlog6：计算 log 的导数。 李代数导数校正  dot(A, B) 就等价于矩阵乘法
        # 阻尼最小二乘求解速度
        JJT = J @ J.T
        JJT.flat[::7] += damp   # 对角加阻尼
        v = -J.T @ np.linalg.solve(JJT, err)
        
        # 流形更新
        q = pin.integrate(model, q, v * DT)
        iteration_nums += 1
    if success :
        positions[:6] = q.tolist()
        return positions
    else :
        return None

def interp_SE3(oM0: pin.SE3, oM1: pin.SE3, s: float) -> pin.SE3:
    if s <= 0.0:
        return oM0
    if s >= 1.0:
        return oM1
    Trel = oM0.actInv(oM1)
    xi = pin.log(Trel)
    return oM0 * pin.exp(xi * s)


def cartesian_linear_trajectory(oM0: pin.SE3, oM1: pin.SE3, T: float, dt: float):
    """Generate N poses from oM0 to oM1 over time T with timestep dt."""
    if T <= 0:
        return [oM1]
    N = max(2, int(np.ceil(T / dt)) + 1)
    traj = [interp_SE3(oM0, oM1, float(i) / (N - 1)) for i in range(N)]
    return traj

def load_robot_config(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config

def get_rebotArm_pos_limit(file_path):
    config = load_robot_config(file_path)
    rebotArm_posmax = [0]*7
    rebotArm_posmin = [0]*7
    for motor_can_id in list(range(1, 8)):
        rebotArm_posmax[motor_can_id-1] = config['joints'][motor_can_id-1]['posmax']
        rebotArm_posmin[motor_can_id-1] = config['joints'][motor_can_id-1]['posmin']
    return rebotArm_posmax,rebotArm_posmin

def q_clip (q):
    rebotArm_posmax,rebotArm_posmin = get_rebotArm_pos_limit(project_root/"config/rebotDM_ik.yaml")    
    q_new = q
    for motor_can_id in list(range(0, 6)):
        if  q_new[motor_can_id] > rebotArm_posmax[motor_can_id] :
            q_new[motor_can_id] = rebotArm_posmax[motor_can_id]
        if  q_new[motor_can_id] < rebotArm_posmin[motor_can_id] :
            q_new[motor_can_id] = rebotArm_posmin[motor_can_id]
    return q_new

def clik_follow(traj, q0, model, data, frame_name="end_link", damp=1e-6, dt=1e-2):
    q = np.array(q0, dtype=float)
    q_hist = []
    q_hist_num = 0
    Max_iteration_nums = 500
    frame_id = model.getFrameId(frame_name)
    for oMdes in traj:
        success = None
        iteration_nums = 0
        while True:
            pin.forwardKinematics(model, data, q)
            pin.pin.updateFramePlacements(model, data)
            iMd = data.oMf[frame_id].actInv(oMdes) # actInv 计算自身的逆并立即作用于目标矩阵
            #compute the err between target and frame 
            err = pin.log(iMd).vector 
            if np.linalg.norm(err) <= 0.01 :   #默认计算 L2 范数，位置和姿态总偏差半径小于 0.01
                success = True
                break
            if iteration_nums >= Max_iteration_nums :
                success = False
                print(iteration_nums)
                break
            J = pin.computeFrameJacobian(model, data, q, frame_id)
            manipulability = np.sqrt(np.linalg.det(J[:6,:] @ J[:6,:].T))
            if manipulability < 0.001:
                print("接近奇异点")  
                return None
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)  #Jlog6：计算 log 的导数。 李代数导数校正  dot(A, B) 就等价于矩阵乘法
            # 阻尼最小二乘求解速度
            JJT = J @ J.T
            JJT.flat[::7] += damp   # 对角加阻尼
            v = -J.T @ np.linalg.solve(JJT, err)
            
            # 流形更新
            q = q_clip (q)
            q = pin.integrate(model, q, v * dt)

            iteration_nums += 1
        if success == True:
            q_hist_num += 1
            q_hist.append(q.tolist())
        elif success == False:
            print("IK compute failed !")
            return None
    return q_hist,q_hist_num



def generate_traj(original_joints_pos,target_pos_point,T,dt,model,data,frame_id = None):
    if frame_id == None :
        frame_id = 7

    q0 = np.array(original_joints_pos)
    frame_name = ["base_link","link1","link2","link3","link4","link5","link6","end_link"]
    frame_id = model.getFrameId(frame_name[frame_id])
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    oM0 = data.oMf[frame_id]


    target_pos = np.array(target_pos_point[0:3])
    rot = pin.rpy.rpyToMatrix(target_pos_point[3], target_pos_point[4], target_pos_point[5])
    oM1 = pin.SE3(rot, target_pos)

    traj = cartesian_linear_trajectory(oM0, oM1, T=T, dt=dt)
    result = clik_follow(traj, q0, model, data, damp=1e-6, dt=0.02)
    if result is None:
        print("Error: clik_follow returned None. IK may have failed.")
        return None, None
    q_traj, q_hist_num = result
    return q_traj , q_hist_num
