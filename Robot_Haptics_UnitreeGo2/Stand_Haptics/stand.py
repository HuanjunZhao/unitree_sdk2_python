import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])

    sport_client = SportClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    sport_client.Sit()
    FR_0 = go2.LegID["FR_0"]
    print(f"[INFO] Joint 'FR_0' pos: {self.channel.low_state.motor_state[FR_0].q}")
    FR_1 = go2.LegID["FR_1"]
    print(f"\n[INFO] Joint 'FR_1' pos: {self.channel.low_state.motor_state[FR_1].q}")
    FR_2 = go2.LegID["FR_2"]
    print(f"[INFO] Joint 'FR_2' pos: {self.channel.low_state.motor_state[FR_2].q} \n")    
    time.sleep(5)
    # sport_client.RiseSit()
    # sport_client.StandUp()
    # time.sleep(2)
