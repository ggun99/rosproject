import time
import sys
import os
import scipy.linalg

import rclpy.time
wd = os.path.abspath(os.getcwd())
sys.path.append(str(wd))
sys.path.append("/home/airlab/Documents/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/programming/zmqRemoteApi/clients/python")
sys.path.append("/home/airlab/robotics_research/mm_controller/coppeliasim")
from cv2 import waitKey
import numpy as np
import rtde_control
import rtde_receive
from jacobian_v1 import Jacobian
from admittance_controller_v1 import AdmittanceController
import rclpy
from rclpy.node import Node 
from scipy.signal import butter, lfilter




class UR5e_controller(Node):
    def __init__(self):
        super().__init__('UR5e_node')
        self.ROBOT_IP = '192.168.0.3'  # 로봇의 IP 주소로 바꿔주세요
        # RTDE Control Interface 초기화
        self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        self.timer = self.create_timer(0.1, self.admittance)

    def admittance(self):
    
        t_start = self.rtde_c.initPeriod()
        # print(f"==>> q_dot.shape: {q_dot.shape}")
        q_ = np.array([0.,0.,0.,0.,0.,0.1])

        acceleration = 0.5
        print('aaa')
        self.rtde_c.speedJ(q_, acceleration, 0.002)
        self.rtde_c.waitPeriod(t_start)
        
def main(args=None):
    rclpy.init(args=args)
    listener = UR5e_controller()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 

    
print('Program started')

if __name__ == '__main__':
    main()
    
