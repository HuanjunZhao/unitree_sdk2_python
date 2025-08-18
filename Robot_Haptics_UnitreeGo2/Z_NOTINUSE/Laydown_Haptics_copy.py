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
import unitree_legged_const as go2

crc = CRC()

class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.low_state = None

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

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def ReadMotorPosition(self, joint_idx :int=0):
        q = self.low_state.motor_state[joint_idx].q
        return q
    

class Go2Leg:
    def __init__(self):
        self.channel = None
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self._startPos = [0.0] * 12

        # Standing pos (need verification) 
        self._standPos = [0, 0.80, -1.50, -0.03, 0.80, -1.50, 
                          0.045063, 0.718109, -1.550600, -0.036850, 0.722423, -1.539088]
        # lay down pos
        # self._layPos = [0, 1.3, -1.30, -0.03, 0.80, -1.50,
        
        # Sitting on foot pos 
        # self._sitonFootPos = [-0.07042285,  1.56507985, -2.84921885, -0.1115451, 1.44777473, -2.82264304, 
        #     -0.003666  ,  2.79740024, -2.83746076,  0.0114518, 2.83350126, -2.89407706]
        # Sitting on foot pos (verified)
        self._sitonFootPos = [0.027953, 1.396641, -1.059938, -0.048694, 1.459891, -1.076525, 
            -0.003666  ,  2.79740024, -2.83746076,  0.0114518, 2.83350126, -2.89407706]
        
        # self._sitPos = [0,  1.65, -1.3,  0,  1.65,
        #                 -1.32681207, -0.11140353,  2.01383797, -2.83593893,  0.13425752,
        #                 2.05665048, -2.89283442]
        self._endingPos = [0.0, 0.0, -0.9, -0.048694, 1.459891, -1.076525, 
            -0.003666  ,  2.79740024, -2.83746076,  0.0114518, 2.83350126, -2.89407706]
        self._tmpPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    @property
    def go2_channel(self):
        return self.channel

    @go2_channel.setter
    def go2_channel(self, channel: Go2Channel):
        self.channel = channel

    def init_cmd(self):
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0]=0xFE
        self.cmd.head[1]=0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        return self.cmd

    def send_cmd(self):
        self.cmd.crc = crc.Crc(self.cmd)
        self.channel.pub.Write(self.cmd)
    
    def init_state(self):
        self.cmd = self.init_cmd()
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.cmd.motor_cmd[i].q= go2.PosStopF
            self.cmd.motor_cmd[i].kp = 0
            self.cmd.motor_cmd[i].dq = go2.VelStopF
            self.cmd.motor_cmd[i].kd = 0
            self.cmd.motor_cmd[i].tau = 0
        self.send_cmd()

    def ready_state(self):
        for joint_idx in range(12):
            self._startPos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)

    def _move_fr_to(self, q_target, t_move=1.2, hz=200, kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move FR joints (0,1,2) to q_target = [q0,q1,q2].
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current FR
        q_now = [self.go2_channel.ReadMotorPosition(0),
                 self.go2_channel.ReadMotorPosition(1),
                 self.go2_channel.ReadMotorPosition(2)]

        n = max(1, int(t_move * hz))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(3)]

            # command FR joints
            for j in range(3):
                i = j  # FR indices are 0,1,2
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_cmd[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0

            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / hz)

        # small hold at the end to settle
        for _ in range(int(0.3 * hz)):
            for j in range(3):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self.send_cmd()
            time.sleep(1.0 / hz)

    def _move_fl_to(self, q_target, t_move=1.2, hz=200, kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move FL joints (3,4,5) to q_target = [q3,q4,q5].
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current FL
        q_now = [self.go2_channel.ReadMotorPosition(3),
                 self.go2_channel.ReadMotorPosition(4),
                 self.go2_channel.ReadMotorPosition(5)]

        n = max(1, int(t_move * hz))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(3)]

            # command FL joints
            for j in range(3):
                i = j + 3  # FL indices are 3,4,5
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_cmd[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0

            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / hz)

        # small hold at the end to settle
        for _ in range(int(0.3 * hz)):
            for j in range(3):
                i = j + 3
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self.send_cmd()
            time.sleep(1.0 / hz)

    def fr_prepare(self,
                      
                      raise_q  = [-0.75, 1.25, -2.75],
                      turn_q  = [0.0, -1.4, -2.75],
                      knock_q = [0, -1.4, -0.9],
                      finished_q = [0.0, -1.4, -2.75],
                      hold_s = 0.15):
        
        # move to raise_q
        print("[INFO] Raising FR...")
        self._move_fr_to(raise_q, t_move=0.5, kp=60.0, kd=5.0)

        # move to turn_q
        print("[INFO] Turning FR...")
        self._move_fr_to(turn_q, t_move=1.6)
        self._move_fr_to(turn_q, t_move=0.3)

        # move to knock_q
        print("[INFO] knocking FR...")
        self._move_fr_to(knock_q, t_move=0.1)


        # brief hold
        t0 = time.time()

        while time.time() - t0 < hold_s:
            for j in range(3):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01

                self.cmd.motor_cmd[i].kp   = 60.0
                self.cmd.motor_cmd[i].kd   = 4.0
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self.send_cmd()
            time.sleep(0.01)

        # move to finished_q
        print("[INFO] Finishing FR...")
        self._move_fr_to(finished_q, t_move=0.8)


if __name__ == '__main__':
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    time.sleep(2.0)

    go2_leg.ready_state()
    print("[INFO] The robot is ready to lay down.")
    time.sleep(2.0)
    go2_leg.fr_prepare()

