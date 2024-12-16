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
from jacobian import Jacobian
from admittance_controller_v1 import AdmittanceController
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node 



class UR5e_controller(Node):
    def __init__(self):
        super().__init__('UR5e_node')
        self.ROBOT_IP = '192.168.0.3'  # 로봇의 IP 주소로 바꿔주세요
        # RTDE 수신 객체 생성
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        # RTDE Control Interface 초기화
        self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        self.dTol = 0.005
        self.derivative_dist = 0.0
        self.maxForce = 100
        self.integral_dist = 0.0
        self.previous_err_dist = 0.0
        self.integral_theta = 0.0
        self.previous_err_theta = 0.0
        self.state = np.array([0, 0, 0, 0, 0, 0]).reshape(-1, 1)
        self.U_previous = None
        self.Sigma_previous = None
        self.Vt_previous = None  
        self.previous_time = time.time() 
        self.Jacobian = Jacobian()
        M = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])  # Mass matrix
        B = np.diag([50.0, 50.0, 50.0, 2.0, 2.0, 2.0])  # Damping matrix
        K = np.diag([500.0, 500.0, 500.0, 5.0, 5.0, 5.0])  # Stiffness matrix
        self.admit = AdmittanceController(M, B, K)     
        self.subscription = self.create_subscription(WrenchStamped, '/ur_ftsensor', self.admittance, 10)

    def admittance(self, msg):
        self.ft = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        t = time.time()
        self.dt = t - self.previous_time
        if self.dt <= 0 or np.isnan(self.dt):
            print("dt가 0이거나 NaN입니다. 기본값으로 설정합니다.")
            self.dt = 1e-6  # 매우 작은 기본값
        self.previous_time = t
        delta_position, _ = self.admit.update(self.ft, self.dt)
        joint_positions = self.rtde_r.getActualQ()
        # joint_velocities = self.rtde_r.getActualQd()
        # joint value (radian)
        j1_r = joint_positions[0]
        j2_r = joint_positions[1]
        j3_r = joint_positions[2]
        j4_r = joint_positions[3]
        j5_r = joint_positions[4]
        j6_r = joint_positions[5]


        delta_position = delta_position.reshape(6,1)
        tcp_pose = self.rtde_r.getActualTCPPose()
        tcp_pose = np.array(tcp_pose).reshape(-1,1)
        # Jacobian matrix
        J = self.Jacobian.jacobian(j1_r, j2_r, j3_r, j4_r, j5_r, j6_r)
        
        j_J = J @ J.T
        
        # SVD 계산
        try:
            U, Sigma, Vt = scipy.linalg.svd(j_J, full_matrices=True)
            self.U_previous, self.Sigma_previous, self.Vt_previous = U, Sigma, Vt  # 이전 값 업데이트
        except ValueError as e:
            print("SVD computation failed due to invalid input:")
            print(e)
            U, Sigma, Vt = self.U_previous, self.Sigma_previous, self.Vt_previous  # 이전 값 사용
        
        epsilon = 1e-6  # 작은 특이값에 대한 임계값
        Sigma_inv = np.diag(1.0 / (Sigma + epsilon))  # 작은 특이값에 임계값을 더하여 안정화
        max_q_dot = 0.1 # 최대 속도 한계를 설정

        # pseudo inverse jacobian matrix
        J_pseudo = Vt.T @ Sigma_inv @ U.T
        print(f"==>> J_pseudo.shape: {J_pseudo.shape}")
        
        d_goal_v = tcp_pose + delta_position
        # print(f"==>> d_goal_v.shape: {d_goal_v.shape}")
        
        q_dot = J_pseudo @ d_goal_v[:3]
        # 속도가 한계를 초과하면 제한
        q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
        q_dot = q_dot.flatten()
        q_dot = np.concatenate((q_dot, [0, 0, 0]))

        
        print(f"==>> q_dot.shape: {q_dot.shape}")

        print(q_dot)
        acceleration = 0.2
        self.rtde_c.speedJ(q_dot, acceleration)
   
        
def main(args=None):
    rclpy.init(args=args)
    listener = UR5e_controller()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 

    
print('Program started')

if __name__ == '__main__':
    main()
    
