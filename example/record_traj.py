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


record_traj=[]
record_q=[]
is_print = False

import threading

# 全局退出标志
exit_program = False
start_record = False
start_return = False
stop_record = True

record_joints_pos = False
start_playback = False

start_record_time = 0
last_record_time = 0
current_time = 0

def ik_compute_joints_pos(traj_list,origin_joints_pos):
    damp = 1e-12
    DT = 1e-1
    positions = [0]*8
    iteration_nums = 0
    Max_iteration_nums = 1000
    frame_id = model.getFrameId("end_link")

    positions[0] = traj_list[0]
    Matrix_t = traj_list[1]
    Matrix_r = traj_list[2]
    oMdes = pin.SE3( Matrix_r, Matrix_t)

    q = np.array(origin_joints_pos[1:7])
    positions[7] = origin_joints_pos[6]
    while True:
        pin.forwardKinematics(model, data, q)
        pin.pin.updateFramePlacements(model, data)
        iMd = data.oMf[frame_id].actInv(oMdes) # actInv 计算自身的逆并立即作用于目标矩阵 
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
            print(f"{origin_joints_pos}接近奇异点")  
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
        positions[1:7] = q.tolist()
        return positions
    else :
        return None





def input_watcher():
    global exit_program,start_record,start_return,stop_record,record_joints_pos,start_playback, start_record_time 
    
    print("\n" * 3)
    print("=" * 30, flush=True)
    print("【提示】输入 help 查询指令", flush=True)
    while True:
        user_input = input("请输入指令: ")  
        if user_input.strip().lower() == "quit":
            exit_program = True
            print("\n机械臂返回初始位置后断开连接", flush=True)
            break  
        elif user_input.strip().lower() == "start":
            start_record = True
            stop_record = False
            start_return = True
            start_record_time = 0
            print("准备录制轨迹，回初始位置中...")

        elif user_input.strip().lower() == "stop":
            start_record = False
            start_return = True
            if stop_record == True :
                print("已经停止录制或者停止运动")
            else :
                stop_record = True
                record_joints_pos = True
                print("停止录制轨迹或者停止运动")

        elif user_input.strip().lower() == "playback":
            print("开始回放")
            start_playback = True

        elif user_input.strip().lower() == "print":
            print("录制轨迹：\n")
            for matrix in record_traj:
                print(matrix)
                print("\n")

        elif user_input.strip().lower() == "help":
            print("quit : 退出程序")
            print("stop : 停止录制轨迹或停止运动")
            print("print : 打印轨迹,格式是【时间戳，平移矩阵，旋转矩阵】")
            print("playback : 开始回放")
        else:
            print(f"未识别指令 '{user_input}'，请输入 help 查询指令", flush=True)
        # print(f"start record : {start_record}")
        # print(f"stop record : {stop_record}")
        # print(f"start return : {start_return}")


if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)

    with reBotArm_handle(ctrl,"rebotDM") as handle:
        if handle.is_connected:
            for motor_can_id in list(range(1,8)) :
                print(f"Motor id {handle.motors[motor_can_id].motor_id}")
                print(f"Motor Mit cfg:{handle.motors[motor_can_id].mit_cfg}")
                print(f"Motor Pos Vel cfg:{handle.motors[motor_can_id].pos_vel_cfg}")
                print(f"Motor Use Modes:{handle.motors[motor_can_id].use_mode}")
                print(f"Motor Pos Max:{handle.motors[motor_can_id].pos_max}")
                print(f"Motor Pos Min:{handle.motors[motor_can_id].pos_min}\n")
            print("Controller is connected and ready.")

            # 启动输入监听线程，设置为守护线程，主程序退出时它自动结束
            input_thread = threading.Thread(target=input_watcher, daemon=True)
            input_thread.start()

        else:
            print("Controller failed to connect.")


        while not exit_program:
            
            if start_record and start_return is False:
                current_time = time.perf_counter()
                if start_record_time == 0:
                    last_record_time = current_time
                    start_record_time = last_record_time
                    record_traj.clear() #录制开始先清空之前录制的轨迹
                    record_q.clear()
                    print("开始录制轨迹")
                    last_Matrix_t, last_Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,model,data)

                elapsed_time = current_time - start_record_time
                if elapsed_time < 60 :
                    joints_pos = handle.return_joints_last_pos()
                    motors_last_pos,full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,model,data)  
                    handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
                    if (current_time - last_record_time) >= 0.1:
                        Matrix_t,Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,model,data)
                        # 上一帧的数据
                        prev_t, prev_r = last_Matrix_t, last_Matrix_r
                        delta_pos = np.linalg.norm(Matrix_t - prev_t)
                        delta_rot = np.linalg.norm(pin.log3(prev_r.T @ Matrix_r))  #对于旋转矩阵，转置等于逆矩阵，log3将旋转矩阵转回向量

                        # 设定阈值 1mm, 5度 也就是0.0435 rad
                        if delta_pos > 0.001 or delta_rot > 0.0435:
                            time_stamp = current_time
                            record_traj.append([time_stamp,Matrix_t.copy(), Matrix_r.copy()])
                            last_Matrix_t, last_Matrix_r = Matrix_t.copy(), Matrix_r.copy()
                            last_record_time = current_time
                else:
                    print("\nWarning: 超过录制时间60s,自动停止录制")
                    print("\n请输入指令 :")
                    stop_record = True
                    start_return = True
                    record_joints_pos = True

            if stop_record:
                time_stamp =  time.perf_counter()
                Matrix_t,Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,model,data)
                record_traj.append([time_stamp,Matrix_t.copy(), Matrix_r.copy()])
                start_record_time = 0
                last_record_time = 0
                start_record = False
                stop_record = False


            if start_return :
                handle.return_zero_position()
                time.sleep(0.5)
                for motor_can_id in list(range(1,7)):
                        handle.motors[motor_can_id].use_mode = "MIT"  
                        handle._change_motor_mode()
                start_return = False

            if record_joints_pos :
                if not record_q:
                    origin_joints_pos = [0, 0,0,0,0,0,0,0]
                    q = origin_joints_pos
                    last_q = q
                    for traj_list in record_traj :
                        q = ik_compute_joints_pos(traj_list,q[1:8])
                        if q:
                            last_q = q
                            record_q.append(q)
                        else:
                            q = last_q
                            q[0] = traj_list[0]
                record_joints_pos = False
                print("保存轨迹记录")

            if start_playback:
                if record_q:
                    for motor_can_id in list(range(1,7)):
                        handle.motors[motor_can_id].use_mode = "POS_VEL"  
                    handle._change_motor_mode()
                    # for q in record_q:
                    #     handle.move_to_joint_positions(positions = q[1:8])
                    #     time.sleep(0.1)
                    for i in range(len(record_q) - 1):
                        t_cur = record_q[i][0]
                        q_cur = record_q[i][1:8]
                        t_next = record_traj[i+1][0]
                        q_next = record_traj[i+1][1:8]
                        handle.move_to_joint_positions(positions = q_cur)
                        time.sleep(t_next - t_cur)
                    print("\n完成回放\n")
                    print("\n请输入指令 :")
                else:
                    print("尚未录制轨迹")
                start_playback = False