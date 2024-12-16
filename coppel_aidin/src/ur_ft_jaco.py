import time
import sys
import os
import scipy.linalg

import rclpy.time
wd = os.path.abspath(os.getcwd())
sys.path.append(str(wd))
sys.path.append("/home/airlab/Documents/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/programming/zmqRemoteApi/clients/python")
sys.path.append("/home/airlab/robotics_research/")
from cv2 import waitKey
import numpy as np
import rtde_control
import rtde_receive
# from mm_controller.coppeliasim.jacobian_v1 import Jacobian
from mm_controller.coppeliasim.admittance_controller_v1 import AdmittanceController
import rclpy
from rclpy.node import Node 
from scipy.signal import butter, lfilter
from pybullet_test.sim_ur5e_bullet import UR5eBullet
import threading

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
        # self.Jacobian = Jacobian()
        M = np.diag([15.0, 15.0, 15.0, 2.0, 2.0, 2.0])  # Mass matrix
        B = np.diag([50.0, 50.0, 50.0, 2.0, 2.0, 2.0])  # Damping matrix
        K = np.diag([100.0, 100.0, 100.0, 5.0, 5.0, 5.0])  # Stiffness matrix
        self.admit = AdmittanceController(M, B, K)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.admittance)
         # ft data list 저장
        self.ft_x = []
        self.ft_y = []
        self.ft_z = []
        self.robot = UR5eBullet("no_gui")
        self.emergency_stop = False
        # Start a thread to listen for emergency stop input
        self.emergency_stop_thread = threading.Thread(target=self.emergency_stop_listener)
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()

    def emergency_stop_listener(self):
        #  """Listen for Enter key to activate emergency stop."""
        print("Press Enter to activate emergency stop.")
        while True:
            input()  # Wait for Enter key press
            self.emergency_stop = True
            self.get_logger().warn("Emergency stop activated!")

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    # 필터 적용 함수
    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    
    def admittance(self):
        t_start = self.rtde_c.initPeriod()
        wrench = self.rtde_r.getActualTCPForce()   # 중력 혹은 다른 힘들이 보정이 된 TCP 에서 측정된 힘
        # wrench = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        TCPpose = self.rtde_r.getActualTCPPose()
        TCPpose = np.array(TCPpose)
        # raw_wrench = self.rtde_r.getFtRawWrench()   # 중력 혹은 다른 힘들이 일체 보정이 되지 않은 raw 데이터
        # 6D 벡터: [Fx, Fy, Fz, Tx, Ty, Tz]
        print("Force/Torque values:", wrench)
        # print("Raw Force/Torque values:", raw_wrench)
        ft = wrench
        ft[3], ft[4], ft[5] = 0, 0, 0
        if not self.rtde_r.isConnected():
            self.rtde_r.reconnect()
        if ft:
            # Filter requirements.
            cutoff = 3.0  # 저역통과 필터의 컷오프 주파수
            fs = 100.0     # 프레임 속도 (초당 프레임)
            order = 3     # 필터 차수
            self.ft_x.append(ft[0])
            self.ft_y.append(ft[1])
            self.ft_z.append(ft[2])
            if len(self.ft_x) > 50 :
                self.ft_x.pop(0)
                self.ft_y.pop(0)
                self.ft_z.pop(0)
            # 데이터가 충분할 때 필터 적용
            if len(self.ft_x) > order:
                filtered_ft_x = self.butter_lowpass_filter(self.ft_x, cutoff, fs, order)
                filtered_ft_y = self.butter_lowpass_filter(self.ft_y, cutoff, fs, order)
                filtered_ft_z = self.butter_lowpass_filter(self.ft_z, cutoff, fs, order)
                ft[0] = filtered_ft_x[-1]
                ft[1] = filtered_ft_y[-1]
                ft[2] = filtered_ft_z[-1]
        t = time.time()

        
        delta_position, _ = self.admit.update(ft, self.dt)
        joint_positions = self.rtde_r.getActualQ()
        print(f"==>> joint_positions: {joint_positions}")
        joint_vel = self.rtde_r.getActualQd()

        # joint velocity value
        joint_vel_arr = np.array([joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3],joint_vel[4],joint_vel[5]])
        # print(f"==>> joint_vel_arr.shape: {joint_vel_arr.shape}")
    
        # new jac
        jac_t, jac_r = self.robot.compute_jac(joint_positions)
        J = np.vstack((jac_t,jac_r))
        j_J = J @ J.T
        # print('linear velocity', np.dot(J, joint_vel_arr))
        
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
        # print(f"==>> J_pseudo.shape: {J_pseudo.shape}")
        d_goal_v = delta_position#*0.1 # /np.linalg.norm(delta_position)
        print(f"==>> d_goal_v: {d_goal_v}")

        # print(f"==>> d_goal_v.shape: {d_goal_v.shape}")
        q_dot = J_pseudo @ d_goal_v

        cal_check = J @ q_dot
        # print('checking:',cal_check)
        # 속도가 한계를 초과하면 제한
        q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
        q_dot = q_dot.flatten()
        # print(f"==>> q_dot.shape: {q_dot.shape}")
        # print(q_dot)
        acceleration = 0.2
        # q_dot_ = q_dot + joint_vel_arr
        print(f"==>> q_dot_: {q_dot}")
        tcppose = TCPpose + d_goal_v
        # joint_pose = self.rtde_c.getInverseKinematics(tcppose)
        # print(f"==>> joint_pose: {joint_pose}")
        if self.emergency_stop:
            q_dot = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        # self.rtde_c.moveJ(joint_pose, 1.0, 0.2)
        # self.rtde_c.speedJ(q_dot, acceleration, self.dt)
        self.rtde_c.waitPeriod(t_start)

def main(args=None):
    rclpy.init(args=args)
    listener = UR5e_controller()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
    listener.rtde_c.servoStop()
    listener.rtde_c.stopScript()


print('Program started')

if __name__ == '__main__':
    waitKey(3000)
    main()

