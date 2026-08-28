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
            print("print : 打印轨迹,格式是【时间戳，关节角度】")
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
                if elapsed_time < 30 :
                    joints_pos = handle.return_joints_last_pos()
                    motors_last_pos,full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,model,data)  
                    handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
                    if (current_time - last_record_time) >= 0.1:
                        time_stamp = current_time
                        record_traj.append([time_stamp,joints_pos])
                        last_record_time = current_time
                else:
                    print("\nWarning: 超过录制时间60s,自动停止录制")
                    print("\n请输入指令 :")
                    stop_record = True
                    start_return = True
                    record_joints_pos = True

            if stop_record:
                time_stamp =  time.perf_counter()
                
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

           
            if start_playback:
                if record_traj:
                    for motor_can_id in list(range(1,7)):
                        handle.motors[motor_can_id].use_mode = "POS_VEL"  
                    handle._change_motor_mode()
                    time.sleep(0.5)
                    for q in record_traj:
                        handle.move_to_joint_positions(positions = q[1])
                        time.sleep(0.1)
                    print("\n完成回放\n")
                    print("\n请输入指令 :")
                else:
                    print("\n尚未录制轨迹")
                    print("\n请输入指令 :")
                start_playback = False