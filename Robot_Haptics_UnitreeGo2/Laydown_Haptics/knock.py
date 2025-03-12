import time
import sys
import math

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import Thread
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import unitree_legged_const as go2

crc = CRC()

class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.low_state = None
        self.duration_readyState = 500
        self.duration_waveLeg = 1000
        self.percent_1 = 0
        self.percent_2 = 0


        if len(ether_name)>1:
            ChannelFactoryInitialize(0, ether_name)
        else:
            ChannelFactoryInitialize(0)

        # Create a publisher to publish the data defined in UserData class
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

        # Create a subscriber to receive the latest robot state every certain seconds 
        self.low_state = None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)      

    # Not used, but usable.
    def LowStateHandler(self, msg: LowState_):    
        # print front right hip motor states
        print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        print("RL_2 motor state: ", msg.motor_state[go2.LegID["RL_2"]])
        print("RL_0 motor state: ", msg.motor_state[go2.LegID["RL_0"]])
        print("IMU state: ", msg.imu_state)
        print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)
        print(f"\n")  

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

class Go2Leg:
    def __init__(self):
        self.channel = None
        self.cmd = unitree_go_msg_dds__LowCmd_()

    @property
    def go2_channel(self):
        return self.channel

    @go2_channel.setter
    def go2_channel(self, channel: Go2Channel):
        self.channel = channel


    def send_cmd(self):
        self.cmd.crc = crc.Crc(self.cmd)

        #Publish message
        if self.channel.pub.Write(self.cmd):
            # print(f"Publish success. msg crc: {self.cmd.crc} \n")
            pass
        else:
            print(f"Waitting for subscriber. \n")

    
    def init_state(self):
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0]=0xFE
        self.cmd.head[1]=0xEF
        self.cmd.level_flag = 0xEE
        self.cmd.gpio = 0

        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.cmd.motor_cmd[i].q= go2.PosStopF
            self.cmd.motor_cmd[i].kp = 0
            self.cmd.motor_cmd[i].dq = go2.VelStopF
            self.cmd.motor_cmd[i].kd = 0
            self.cmd.motor_cmd[i].tau = 0

        self.send_cmd()

    def joint_linear_interpolation(self, init_pos: float, target_pos: float, rate: float):
        p = 0.0
        rate = min(max(rate, 0.0), 1.0)
        p = init_pos * (1.0 - rate) + target_pos * rate
        return p

                  


    def Knock(self):
        q_init = [0.0, -1.5, -2.8]
        q_des =  [0.0, 0.0, 0.0]
        
        # sim_mid_q = [0.0, -1.5, -1] # light
        sim_mid_q = [0.0, -1.5, 0] # normal
        # sim_mid_q = [0.0, -1.5, 1] # smash
        moter_kp = 60.0
        for rate_count in range(10, 400):
            rate = float(rate_count) / 200.0

            for joint_idx in range(3):
                q_des[joint_idx] = self.joint_linear_interpolation(q_init[joint_idx], sim_mid_q[joint_idx], rate)
            
            for joint_idx in range(3):
                joint_offset = go2.LegID["FR_0"]
                i = joint_offset + joint_idx
                # print("FR_0 motor state: ", self.channel.low_state.motor_state[go2.LegID["FR_0"]])
                # print("FR_0 motor state: ", go2.LegID["FR_0"])
                self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[i].q= q_des[joint_idx]
                self.cmd.motor_cmd[i].kp = moter_kp
                self.cmd.motor_cmd[i].dq = 0.0 # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[i].kd = 1.0
                self.cmd.motor_cmd[i].tau = 0.0
                time.sleep(0.1)
                self.send_cmd()                
            
            print(f"\n")
        self.cmd.motor_cmd[go2.LegID["FR_0"]].tau = 5.0
        self.send_cmd()
        FR_0 = go2.LegID["FR_0"]
        print(f"\n[INFO] Joint 'FR_0' pos: {self.channel.low_state.motor_state[FR_0].q}")
        FR_1 = go2.LegID["FR_1"]
        print(f"\n[INFO] Joint 'FR_1' pos: {self.channel.low_state.motor_state[FR_1].q}")
        FR_2 = go2.LegID["FR_2"]
        print(f"\n[INFO] Joint 'FR_2' pos: {self.channel.low_state.motor_state[FR_2].q} \n")                     
  
if __name__ == '__main__':

    # 1. Create comm channel
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()


    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    time.sleep(2.0)

    go2_leg.Knock()

    time.sleep(2.0)
