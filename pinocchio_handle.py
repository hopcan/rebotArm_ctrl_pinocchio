import pinocchio as pin
import numpy as np



def gravity_conpensation_control(joints_pos,model,data) :
    motors_last_pos = joints_pos
    full_torques = np.zeros(7)

    #compute gravity
    pin.computeGeneralizedGravity(model, data,  np.array(motors_last_pos[:6]))
    torque = data.g
    full_torques[:6] = torque 

    #add conpensation
    full_torques[3] *= 2.5   

    return motors_last_pos,full_torques




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
        if np.linalg.norm(err) <= 0.01 :
            success = True
            break
        if iteration_nums >= Max_iteration_nums :
            success = False
            break
        J = pin.computeFrameJacobian(model, data, q, frame_id)
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)  #Jlog6：计算 log 的导数。
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