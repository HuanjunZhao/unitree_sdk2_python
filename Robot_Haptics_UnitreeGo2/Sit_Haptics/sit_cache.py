# # # import time
# # # import sys
# # # from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
# # # from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
# # # from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
# # # from unitree_sdk2py.go2.sport.sport_client import (
# # #     SportClient,
# # #     PathPoint,
# # #     SPORT_PATH_POINT_SIZE,
# # # )

# # # import unitree_legged_const as go2

# # # class Go2:
# # #     def __init__(self):
# # #         self.channel = None
# # #         self.cmd = unitree_go_msg_dds__LowCmd_()

# # # class Go2Channel:
# # #     def __init__(self, ether_name: str=""):
# # #         self.pub = None
# # #         self.sub = None
# # #         self.low_state = None
# # #         self.duration_readyState = 500
# # #         self.duration_waveLeg = 1000
# # #         self.percent_1 = 0
# # #         self.percent_2 = 0


# # #         if len(ether_name)>1:
# # #             ChannelFactoryInitialize(0, ether_name)
# # #         else:
# # #             ChannelFactoryInitialize(0)

# # #         # Create a publisher to publish the data defined in UserData class
# # #         self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
# # #         self.pub.Init()

# # #         # Create a subscriber to receive the latest robot state every certain seconds 
# # #         self.low_state = None
# # #         self.sub = ChannelSubscriber("rt/lowstate", LowState_)
# # #         self.sub.Init(self.LowStateMessageHandler, 10)      


# # # if __name__ == "__main__":

# # #     if len(sys.argv) < 2:
# # #         print(f"Usage: python3 {sys.argv[0]} networkInterface")
# # #         sys.exit(-1)
    
# # #     if len(sys.argv)>1:
# # #         go2_channel = Go2Channel(sys.argv[1])
# # #     else:
# # #         go2_channel = Go2Channel()

# # #     ChannelFactoryInitialize(0, sys.argv[1])

# # #     sport_client = SportClient()  
# # #     sport_client.SetTimeout(10.0)
# # #     sport_client.Init()

# # #     sport_client.Sit()


# # #     go2_leg = Go2()
# # #     go2_leg.go2_channel = go2_channel
# # #     go2_leg.init_state()

# # #     time.sleep(5)
# # #     # sport_client.RiseSit()
# # #     # sport_client.StandUp()
# # #     # time.sleep(2)

# import time
# import sys
# import math

# from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
# from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
# from unitree_sdk2py.utils.crc import CRC
# from unitree_sdk2py.utils.thread import Thread
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
# from unitree_sdk2py.go2.sport.sport_client import (
#     SportClient,
#     PathPoint,
#     SPORT_PATH_POINT_SIZE,
# )
# import unitree_legged_const as go2

# crc = CRC()

# class Go2Channel:
#     def __init__(self, ether_name: str=""):
#         self.pub = None
#         self.sub = None
#         self.low_state = None
#         self.duration_readyState = 500
#         self.duration_waveLeg = 1000
#         self.percent_1 = 0
#         self.percent_2 = 0


#         if len(ether_name)>1:
#             ChannelFactoryInitialize(0, ether_name)
#         else:
#             ChannelFactoryInitialize(0)

#         # Create a publisher to publish the data defined in UserData class
#         self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
#         self.pub.Init()

#         # Create a subscriber to receive the latest robot state every certain seconds 
#         self.low_state = None
#         self.sub = ChannelSubscriber("rt/lowstate", LowState_)
#         self.sub.Init(self.LowStateMessageHandler, 10)      

#     # Not used, but usable.
#     def LowStateHandler(self, msg: LowState_):    
#         # print front right hip motor states
#         print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
#         print("RL_2 motor state: ", msg.motor_state[go2.LegID["RL_2"]])
#         print("RL_0 motor state: ", msg.motor_state[go2.LegID["RL_0"]])
#         print("IMU state: ", msg.imu_state)
#         print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)
#         print(f"\n")  

#     def LowStateMessageHandler(self, msg: LowState_):
#         self.low_state = msg

# class Go2Leg:
#     def __init__(self):
#         self.channel = None
#         self.cmd = unitree_go_msg_dds__LowCmd_()

#     @property
#     def go2_channel(self):
#         return self.channel

#     @go2_channel.setter
#     def go2_channel(self, channel: Go2Channel):
#         self.channel = channel


#     def send_cmd(self):
#         self.cmd.crc = crc.Crc(self.cmd)

#         #Publish message
#         if self.channel.pub.Write(self.cmd):
#             # print(f"Publish success. msg crc: {self.cmd.crc} \n")
#             pass
#         else:
#             print(f"Waitting for subscriber. \n")

    
#     def init_state(self):
#         self.cmd = unitree_go_msg_dds__LowCmd_()
#         self.cmd.head[0]=0xFE
#         self.cmd.head[1]=0xEF
#         self.cmd.level_flag = 0xEE
#         self.cmd.gpio = 0

#         for i in range(20):
#             self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
#             self.cmd.motor_cmd[i].q= go2.PosStopF
#             self.cmd.motor_cmd[i].kp = 0
#             self.cmd.motor_cmd[i].dq = go2.VelStopF
#             self.cmd.motor_cmd[i].kd = 0
#             self.cmd.motor_cmd[i].tau = 0

#         self.send_cmd()

#     def joint_linear_interpolation(self, init_pos: float, target_pos: float, rate: float):
#         p = 0.0
#         rate = min(max(rate, 0.0), 1.0)
#         p = init_pos * (1.0 - rate) + target_pos * rate
#         return p

#     def Knock(self):
#         q_init = [0.0, -1.5, -2.8]
#         q_des =  [0.0, 0.0, 0.0]
        
#         sim_mid_q = [0.0, -1.5, -1.1]

#         for rate_count in range(10, 400):
#             rate = float(rate_count) / 200.0

#             for joint_idx in range(3):
#                 q_des[joint_idx] = self.joint_linear_interpolation(q_init[joint_idx], sim_mid_q[joint_idx], rate)
            
#             for joint_idx in range(3):
#                 joint_offset = go2.LegID["FR_0"]
#                 i = joint_offset + joint_idx
#                 # print("FR_0 motor state: ", self.channel.low_state.motor_state[go2.LegID["FR_0"]])
#                 # print("FR_0 motor state: ", go2.LegID["FR_0"])
#                 self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
#                 self.cmd.motor_cmd[i].q= q_des[joint_idx]
#                 self.cmd.motor_cmd[i].kp = 20.0
#                 self.cmd.motor_cmd[i].dq = 0.0 # Set to stop angular velocity(rad/s)
#                 self.cmd.motor_cmd[i].kd = 1.0
#                 self.cmd.motor_cmd[i].tau = 0.0

#                 self.send_cmd()                
            
#             print(f"\n")
#         self.cmd.motor_cmd[go2.LegID["FR_0"]].tau = 5.0
#         self.send_cmd()
#         FR_0 = go2.LegID["FR_0"]
#         print(f"\n[INFO] Joint 'FR_0' pos: {self.channel.low_state.motor_state[FR_0].q}")
#         FR_1 = go2.LegID["FR_1"]
#         print(f"\n[INFO] Joint 'FR_1' pos: {self.channel.low_state.motor_state[FR_1].q}")
#         FR_2 = go2.LegID["FR_2"]
#         print(f"\n[INFO] Joint 'FR_2' pos: {self.channel.low_state.motor_state[FR_2].q} \n")                     

#     def getJointPos(self):
#         print(f"[INFO] Joint 'FR_0' pos: {go2_leg.channel.low_state.motor_state[0].q}")        
#         print(f"[INFO] Joint 'FR_1' pos: {go2_leg.channel.low_state.motor_state[1].q}")        
#         print(f"[INFO] Joint 'FR_2' pos: {go2_leg.channel.low_state.motor_state[2].q} \n\n")  

#         print(f"[INFO] Joint 'FL_0' pos: {go2_leg.channel.low_state.motor_state[3].q}")        
#         print(f"[INFO] Joint 'FL_1' pos: {go2_leg.channel.low_state.motor_state[4].q}")        
#         print(f"[INFO] Joint 'FL_2' pos: {go2_leg.channel.low_state.motor_state[5].q} \n\n") 

#         print(f"[INFO] Joint 'RR_0' pos: {go2_leg.channel.low_state.motor_state[6].q}")        
#         print(f"[INFO] Joint 'RR_1' pos: {go2_leg.channel.low_state.motor_state[7].q}")        
#         print(f"[INFO] Joint 'RR_2' pos: {go2_leg.channel.low_state.motor_state[8].q} \n\n") 

#         print(f"[INFO] Joint 'RL_0' pos: {go2_leg.channel.low_state.motor_state[9].q}")        
#         print(f"[INFO] Joint 'RL_0' pos: {go2_leg.channel.low_state.motor_state[10].q}")        
#         print(f"[INFO] Joint 'RL_0' pos: {go2_leg.channel.low_state.motor_state[11].q} \n\n") 

#         time.sleep(5.0)                   
#         return 0

# if __name__ == '__main__':

#     # 1. Create comm channel
#     if len(sys.argv)>1:
#         go2_channel = Go2Channel(sys.argv[1])
#     else:
#         go2_channel = Go2Channel()


#     sport_client = SportClient() 
#     sport_client.Init()
#     sport_client.SetTimeout(10.0)

#     # sport_client.Sit()  
#     # sport_client.StandDown()  
#     sport_client.StandUp()
#     time.sleep(3.0)  

#     # # 2. Initialize the dog with init state
#     go2_leg = Go2Leg()
#     go2_leg.go2_channel = go2_channel
#     go2_leg.init_state()
#     while True:
#         getJointPos = go2_leg.getJointPos()
       
#     time.sleep(2.0)

import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Custom:
    def __init__(self):
        self.Kp = 60.0
        self.Kd = 5.0
        self.dt = 0.002  # 控制周期
        
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  

        # 站立姿势
        self._standPos =  [0.025963, 0.802967, -1.507695,  -0.030346, 0.819930, -1.503292,  # FR_0, FR_1, FR_2 (前右) # FL_0, FL_1, FL_2 (前左)
                           0.045063, 0.718109, -1.550600, -0.036850, 0.722423, -1.539088] # RR_0, RR_1, RR_2 (后右) # RL_0, RL_1, RL_2 (后左)

        self._sitPos = [0.027953, 1.396641, -1.059938,  # FR_0, FR_1, FR_2 (前右)
                        -0.048694, 1.459891, -1.076525,  # FL_0, FL_1, FL_2 (前左)
                        -0.012320, 2.025182, -2.797226,  # RR_0, RR_1, RR_2 (后右)
                        0.024008, 2.036127, -2.832129   # RL_0, RL_1, RL_2 (后左)
]
        self.startPos = [0.0] * 12
        self.duration = 500  # 过渡时间
        self.percent = 0
        
        self.firstRun = True
        self.done = False
        
        self.lowCmdWriteThreadPtr = None
        self.crc = CRC()

    def Init(self):
        self.InitLowCmd()
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        
        status, result = self.msc.CheckMode()
        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.002, target=self.LowCmdWrite, name="writesitcmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def LowCmdWrite(self):
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        self.percent += 1.0 / self.duration
        self.percent = min(self.percent, 1)

        for i in range(12):
            self.low_cmd.motor_cmd[i].q = (1 - self.percent) * self._standPos[i] + self.percent * self._sitPos[i]
            self.low_cmd.motor_cmd[i].dq = 0
            self.low_cmd.motor_cmd[i].kp = self.Kp
            self.low_cmd.motor_cmd[i].kd = self.Kd
            self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

        if self.percent == 1.0:
            time.sleep(1)
            print("Sitting pose reached!")
            sys.exit(-1)

if __name__ == '__main__':
    print("WARNING: Ensure there are no obstacles around the robot.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:
        time.sleep(1)
