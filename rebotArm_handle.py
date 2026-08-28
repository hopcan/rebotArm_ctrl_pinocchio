from motorbridge import Controller,Mode
import time
import yaml
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
rebotDM_file_path = os.path.join(base_dir, 'config', 'rebotDM.yaml')
rebotRS_file_path = os.path.join(base_dir, 'config', 'rebotRS.yaml')


from dataclasses import dataclass
from typing import Optional, Any

@dataclass
class Motor:
    motor_id: int
    handle: Optional[Any] = None
    mit_cfg: Optional[dict] = None
    pos_vel_cfg: Optional[dict] = None
    use_mode: Optional[str] = None
    pos_max: float = 0.0
    pos_min: float = 0.0
    state: Optional[dict] = None


class reBotArm_handle:

    def __init__(self, interface,arm_version = "rebotDM",config_path = None):  #rebotDM or rebotRS
        self.is_connected = False
        self.is_all_ids_valid = False
        self.ctrl = interface
        self.expected_ids = list(range(1, 8))

        self.motors = {
            motor_id: Motor(motor_id=motor_id)
            for motor_id in self.expected_ids
        }

        self.arm_version = arm_version
        if config_path == None :
            if self.arm_version == "rebotDM" :
                self.file_path = rebotDM_file_path 
            elif self.arm_version == "rebotRS" :
                self.file_path = rebotRS_file_path
        else :
            self.file_path = config_path
    def _load_robot_config(self, file_path=None):
        if file_path is None:
            file_path = self.file_path
        with open(file_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config

    def _check_is_cfg_valid(self):
        config = self._load_robot_config() 
        for motor_can_id in  list(range(0, 7)):
            if config['joints'][motor_can_id]['MIT']["kp"] == None or config['joints'][motor_can_id]['MIT']["kd"] == None:
                print(f"\nMIT Configuration for motor {motor_can_id + 1} is invalid.Please check your configure yaml.")
                return False
            if config['joints'][motor_can_id]['POS_VEL'] ["vel_kp"]is None or config['joints'][motor_can_id ]['POS_VEL'] ["vel_ki"] is None :
                print(f"\nPOS_VEL Pos Configuration for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False
            if config['joints'][motor_can_id]['POS_VEL'] ["vel_kp"]is None or config['joints'][motor_can_id]['POS_VEL'] ["vel_ki"] is None :
                print(f"\nPOS_VEL Vel Configuration for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False
            if config['joints'][motor_can_id]['POS_VEL'] ["vlim"] is None :
                print(f"\nPOS_VEL Vlim Configuration for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False
            if config['joints'][motor_can_id ]['posmax'] is None :
                print(f"\nposmax for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False
            if config['joints'][motor_can_id]['posmin'] is None:
                print(f"\nposmin for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False 
            if config['joints'][motor_can_id]['use_mode'] is None:
                print(f"\nuse_mode for motor {motor_can_id + 1} is invalid.Please check your configure yaml")
                return False
        return config

    def _add_motor_to_ctrl(self): #add motor and Configuration to controller
        config = self._check_is_cfg_valid() 
        if config == False:
            return False
        for motor_can_id in list(range(1, 8)):
            
            if(self.arm_version=="rebotDM"):
                motor_master_id = 0x10 + motor_can_id
                try:
                    self.motors[motor_can_id].handle = self.ctrl.add_damiao_motor(motor_can_id, motor_master_id, "4340P")
                except Exception as e:
                    print(f"[error] motor_can_id=0x{motor_can_id:02X}: {e}")
                    return False
            elif(self.arm_version=="rebotRS"):
                motor_master_id = 0xfd
                try:
                    self.motors[motor_can_id].handle = self.ctrl.add_robstride_motor(motor_can_id, motor_master_id, "rs-06")
                except Exception as e:
                    print(f"[error] motor_can_id=0x{motor_can_id:02X}: {e}")
                    return False
            self.motors[motor_can_id].mit_cfg = config['joints'][motor_can_id-1]['MIT']
            self.motors[motor_can_id].pos_vel_cfg = config['joints'][motor_can_id-1]['POS_VEL']
            self.motors[motor_can_id].use_mode = config['joints'][motor_can_id-1]['use_mode']
            self.motors[motor_can_id].pos_max = config['joints'][motor_can_id-1]['posmax']
            self.motors[motor_can_id].pos_min = config['joints'][motor_can_id-1]['posmin']
        
        return True
    
    def check_ids_valid(self):
        found_motors=[]
        for motor_can_id in list(range(1, 8)):
                if self.arm_version=="rebotDM":
                    motor_master_id = 0x10 + motor_can_id
                    print(f"Checking ID 0x{motor_can_id:02X} ({len(found_motors)+1}/{len(self.expected_ids)})")
                    try:
                        can_id = self.motors[motor_can_id].handle.get_register_u32(8, timeout_ms=100)
                        master_id = self.motors[motor_can_id].handle.get_register_u32(7, timeout_ms=100)
                        #check if the can_id and master_id match the expected values
                        if can_id == motor_can_id and master_id == motor_master_id :
                            found_motors.append(can_id)
                        elif master_id !=motor_master_id:
                            print(f"can id: {can_id:02x} mismatch, offset is wrong")
                            self.motors[motor_can_id].handle.close()
                            return False  
                    except Exception:
                        # read error, no this can id
                        print(f"[no respond] motor_can_id=0x{motor_can_id:02X}")
                        return False
                    
                elif self.arm_version=="rebotRS":
                    print(f"Checking ID 0x{motor_can_id:02X} ({len(found_motors)+1}/{len(self.expected_ids)})")
                    try:
                        can_id, respond_id = self.motors[motor_can_id].handle.robstride_ping()
                        if respond_id == 254 and can_id != None:
                            found_motors.append(can_id)   
                        else :
                            self.motors[motor_can_id].handle.close()
                            return False  
                    except Exception:
                        # scan error
                        print(f"[no respond] no this motor_can_id=0x{motor_can_id:02X}")
         
        if len(found_motors) == len(self.expected_ids) :
            print("All ids are valid !")
            return True
        
    def get_joints_state(self):
        joints_pos = [0]*7
        for motor_can_id in list(range(1, 8)):
            self.motors[motor_can_id].handle.request_feedback()
            time.sleep(0.01)
            self.motors[motor_can_id].state = self.motors[motor_can_id].handle.get_state()
            joints_pos[motor_can_id-1] = self.motors[motor_can_id].state.pos
        return joints_pos

    def return_joints_last_pos(self):
        joints_pos = [0]*7
        for motor_can_id in list(range(1, 8)):
            self.motors[motor_can_id].state = self.motors[motor_can_id].handle.get_state()
            joints_pos[motor_can_id-1] = self.motors[motor_can_id].state.pos
        return joints_pos

    def connect(self):
        if self._add_motor_to_ctrl():
            if self.check_ids_valid() :
                try :
                    self.ctrl.enable_all()
                    if self._change_motor_mode() :
                        for motor_can_id in list(range(1,8)):
                            self.motors[motor_can_id].state = self.motors[motor_can_id].handle.get_state()
                        return True
                    else :
                        return False
                except Exception:
                    print("enable all motors failed")
                    return False
            else:
                print("Not all IDs are valid. Cannot connect.")
                return False
        else :
            print("Failed to add motors to controller. Cannot connect.")
            return False
        
    def disconnect(self):
        if self.is_connected:
            self.return_zero_position()
            self.ctrl.disable_all()
            self.ctrl.close_bus()
            self.ctrl.close()
            self.is_connected = False
            print("\n\nrebot disconnected.\n\n")
        else :
            print("\n\nrebot is not connected.\n\n")


    def set_zero_position(self):
        self.ctrl.disable_all()
        for motor_can_id in list(range(1, 8)):
            try:
                self.motors[motor_can_id].handle.set_zero_position()
                time.sleep(0.2)
            except Exception:
                print(f"motor {motor_can_id}: set zero failed")
                return False
            self.motors[motor_can_id].handle.request_feedback()
            state = self.motors[motor_can_id].handle.get_state()
            if state.pos < 0.1 and state.pos > -0.1:
                print(f"motor {motor_can_id}: set zero successfully")
            else :
                print(f"motor {motor_can_id}: set zero failed")
        self.ctrl.enable_all()

    def _change_motor_mode(self):
        for motor_can_id in list(range(1, 8)):
            if self.motors[motor_can_id].use_mode is None:
               self.motors[motor_can_id].use_mode = "MIT"  # Default to MIT mode if not specified
            if self.motors[motor_can_id].use_mode == "MIT":
                try:
                    self.motors[motor_can_id].handle.ensure_mode(Mode.MIT, timeout_ms=1000)
                    # print(f"motor {motor_can_id}: change mode to MIT successfully")
                except Exception:
                    print(f"motor {motor_can_id}: change mode to MIT failed")
                    return False
            elif self.motors[motor_can_id].use_mode == "POS_VEL":
                try:
                    self.motors[motor_can_id].handle.ensure_mode(Mode.POS_VEL, timeout_ms=1000)
                    # print(f"motor {motor_can_id}: change mode to POS_VEL successfully")
                except Exception:
                    print(f"motor {motor_can_id}: change mode to POS_VEL failed")
                    return False
            
        return True
    
    def move_to_joint_positions(self,positions,velocity=None,torque=None):
        if not self.is_connected:
            return False
        if len(positions) != len(self.expected_ids):
            print("Positions list length does not match number of motors.")
            return False
        for motor_can_id in list(range(1, 8)):
            if  positions[motor_can_id-1] > self.motors[motor_can_id].pos_max :
                positions[motor_can_id-1] = self.motors[motor_can_id].pos_max
            if  positions[motor_can_id-1] < self.motors[motor_can_id].pos_min:
                positions[motor_can_id-1] = self.motors[motor_can_id].pos_min
            if  self.motors[motor_can_id].use_mode == "MIT":
                if velocity is None:
                    vel= 0.0
                else :
                    vel = velocity[motor_can_id-1]
                if torque is None:
                    tau = 0.0
                else :
                    tau = torque[motor_can_id-1]
            if self.arm_version == "rebotRS":
                if self.motors[motor_can_id].use_mode == "MIT" :
                    self.motors[motor_can_id].handle.send_mit(
                        pos=positions[motor_can_id-1]*-1,    
                        vel=vel*-1,
                        kp=self.motors[motor_can_id].mit_cfg["kp"],    
                        kd=self.motors[motor_can_id].mit_cfg["kd"],    
                        tau=tau*-1   
                    )
                elif self.motors[motor_can_id].use_mode == "POS_VEL" :    
                    self.motors[motor_can_id].handle.send_pos_vel(
                        pos=positions[motor_can_id-1]*-1,    # target angle（rad）
                        vlim=1.5    # max vel（rad/s）
                    )
            elif self.arm_version == "rebotDM":
                if self.motors[motor_can_id].use_mode == "MIT" :
                    self.motors[motor_can_id].handle.send_mit(
                    pos=positions[motor_can_id-1],    
                    vel=vel,
                    kp=self.motors[motor_can_id].mit_cfg["kp"],    
                    kd=self.motors[motor_can_id].mit_cfg["kd"],    
                    tau=tau  
                )
                elif self.motors[motor_can_id].use_mode == "POS_VEL":    
                    self.motors[motor_can_id].handle.send_pos_vel(
                        pos=positions[motor_can_id-1],    # target angle（rad）
                        vlim=1.5    # max vel（rad/s）
                    )
            self.motors[motor_can_id].state = self.motors[motor_can_id].handle.get_state()
        return True

    def return_zero_position(self):
        dt = 0.01  # 10ms 
        target_pos=[0,0,-0.3,-0.1,0,0,0]
        start_time=time.perf_counter()
        while time.perf_counter()- start_time < 3.0:
            if self.arm_version == "rebotDM" :
                for motor_can_id in [1,2,3]:
                    if  self.motors[motor_can_id].use_mode == "MIT":
                        self.motors[motor_can_id].handle.send_mit(
                            pos=target_pos[motor_can_id - 1] ,    
                            vel=0.0,
                            kp=20.0 ,   
                            kd=12.0,    
                            tau=0.0   
                        )
                    if  self.motors[motor_can_id].use_mode == "POS_VEL":
                        self.motors[motor_can_id].handle.send_pos_vel(
                            pos=0.0,    # target angle（rad）
                            vlim=1.5    # max vel（rad/s）
                        )
                    time.sleep(dt)
                for motor_can_id in [4,5,6,7]:
                    if  self.motors[motor_can_id].use_mode == "MIT":
                        self.motors[motor_can_id].handle.send_mit(
                            pos=target_pos[motor_can_id - 1] ,    
                            vel=0.0,
                            kp=20.0 ,   
                            kd=12.0,    
                            tau=0.0   
                        )
                    if  self.motors[motor_can_id].use_mode == "POS_VEL":
                        self.motors[motor_can_id].handle.send_pos_vel(
                            pos=0.0,    # target angle（rad）
                            vlim=1.5    # max vel（rad/s）
                        )
            elif self.arm_version == "rebotRS" :
                for motor_can_id in [1,2,3]:
                    if  self.motors[motor_can_id].use_mode == "MIT":
                        self.motors[motor_can_id].handle.send_mit(
                            pos=-1*target_pos[motor_can_id - 1] ,    
                            vel=0.0,
                            kp=10.0 ,   
                            kd=6.0,    
                            tau=0.0   
                        )
                    if  self.motors[motor_can_id].use_mode == "POS_VEL":
                        self.motors[motor_can_id].handle.send_pos_vel(
                            pos=0.0,    # target angle（rad）
                            vlim=1.5    # max vel（rad/s）
                        )
                    time.sleep(dt)
                for motor_can_id in [4,5,6,7]:
                    if  self.motors[motor_can_id].use_mode == "MIT":
                        self.motors[motor_can_id].handle.send_mit(
                            pos=-1*target_pos[motor_can_id - 1],    
                            vel=0.0,
                            kp=60.0 ,   
                            kd=15.0,    
                            tau=0.0   
                        )
                    if  self.motors[motor_can_id].use_mode == "POS_VEL":
                        self.motors[motor_can_id].handle.send_pos_vel(
                            pos=0.0,    # target angle（rad）
                            vlim=1.5    # max vel（rad/s）
                        )
                time.sleep(dt)


    def __enter__(self):
        if self.connect():
            self.is_connected = True
        else:
            self.is_connected = False
        return self
    
    def __exit__(self, *args):
        self.disconnect()

if __name__ == "__main__":
    channel = "/dev/ttyACM0"  
    ctrl =Controller.from_dm_serial(channel, 921600)
    # channel = "can0"
    # ctrl = Controller(channel)
    with reBotArm_handle(ctrl,"rebotDM") as handle:
        if handle.is_connected:
            print("Controller is connected and ready.")
            print("Motor Use Modes:", handle.use_mode)
        else:
            print("Controller failed to connect.")
        # handle.set_zero_position()
        while True:
            handle.move_to_joint_positions([0,0,0,0,0,0,0])
            time.sleep(0.002)

