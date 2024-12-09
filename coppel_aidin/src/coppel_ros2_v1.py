from cv2 import threshold
import rclpy
from rclpy.node import Node 
import sys
import os
import time
import numpy as np
import scipy.linalg
from geometry_msgs.msg import WrenchStamped
wd = os.path.abspath(os.getcwd())
sys.path.append(str(wd))
sys.path.append("/home/airlab/robotics_research/mm_controller/coppeliasim")
sys.path.append("/home/airlab/Documents/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/programming/zmqRemoteApi/clients/python")
from cv2 import waitKey
from zmqRemoteApi import RemoteAPIClient
from kalman import Kalman
from jacobian import Jacobian
from coppel_controller import coppel_controller
from admittance_controller import AdmittanceController
# from coppel_MM_jaco_kalman_class import Coppeliasim

class Coppel_Ros2(Node):
    def __init__(self):
       super().__init__('coppeliasim_node')
       # Define system parameters
       self.M = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])  # Mass matrix
       self.B = np.diag([20.0, 20.0, 20.0, 2.0, 2.0, 2.0])  # Damping matrix
       self.K = np.diag([50.0, 50.0, 50.0, 5.0, 5.0, 5.0])  # Stiffness matrix
       self.ftSub = self.create_subscription(WrenchStamped, '/aidin_ftsensor',self.ft_process, 10)
       self.Kalman = Kalman()
       self.Jacobian = Jacobian()
       self.coppel_simulator = coppel_controller()
       self.client = RemoteAPIClient()
       self.sim = self.client.getObject('sim')
    #    self.coppel = Coppeliasim(self.sim)
       self.client.step()
       self.sim.startSimulation()
       self.previous_time = 0
       self.X_d_list = []
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
       self.sim = self.sim
       self.dummy = self.sim.getObject('/Dummy')
       self.f_L = self.sim.getObjectHandle('/base_link_respondable/front_left_wheel')
       self.f_R = self.sim.getObjectHandle('/base_link_respondable/front_right_wheel')
       self.r_L = self.sim.getObjectHandle('/base_link_respondable/rear_left_wheel')
       self.r_R = self.sim.getObjectHandle('/base_link_respondable/rear_right_wheel')
       self.Ur5_EE = self.sim.getObject('/base_link_respondable/UR5/joint/link/joint/link/joint/link/joint/link/joint/link/joint/link/connection')
       self.j1 = self.sim.getObject('/base_link_respondable/UR5/joint')
       self.j2 = self.sim.getObject('/base_link_respondable/UR5/joint/joint')
       self.j3 = self.sim.getObject('/base_link_respondable/UR5/joint/joint/joint')
       self.j4 = self.sim.getObject('/base_link_respondable/UR5/joint/joint/joint/joint')
       self.j5 = self.sim.getObject('/base_link_respondable/UR5/joint/joint/joint/joint/joint')
       self.j6 = self.sim.getObject('/base_link_respondable/UR5/joint/joint/joint/joint/joint/joint')
       self.j1_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint')
       self.j2_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint/joint')
       self.j3_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint/joint/joint')
       self.j4_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint/joint/joint/joint')
       self.j5_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint/joint/joint/joint/joint')
       self.j6_h = self.sim.getObjectHandle('/base_link_respondable/UR5/joint/joint/joint/joint/joint/joint')

    def moveToAngle_RL(self, w_R, w_L):
        vel_r = w_R
        vel_l = w_L
        self.sim.setJointTargetVelocity(self.f_L, vel_l)
        self.sim.setJointTargetVelocity(self.f_R, -vel_r)
        self.sim.setJointTargetVelocity(self.r_L, vel_l)
        self.sim.setJointTargetVelocity(self.r_R, -vel_r)

    def set_velocity(self, l_v, a_v, j1_tv, j2_tv, j3_tv, j4_tv, j5_tv, j6_tv, coppel_controller):
        w_R, w_L, wc = coppel_controller.coppel_scout2_controller(l_v, a_v)

        self.sim.setJointTargetVelocity(self.j1_h, j1_tv)  #-wc*angle_con_value)
        self.sim.setJointTargetVelocity(self.j2_h, j2_tv)
        self.sim.setJointTargetVelocity(self.j3_h, j3_tv)
        self.sim.setJointTargetVelocity(self.j4_h, j4_tv)
        self.sim.setJointTargetVelocity(self.j5_h, j5_tv)
        self.sim.setJointTargetVelocity(self.j6_h, j6_tv)
        return w_R, w_L, wc

    def coppeliasim(self, Kalman,Jacobian, dt, X_d_list, coppel_controller):
        covariance, A, B, Q, H, R = Kalman.kalman_init(dt)

        # s = f'Simulation time: {t:.2f} [s]'
        # print(s)
        # joint value (radian)
        j1_r = self.sim.getJointPosition(self.j1)
        j2_r = self.sim.getJointPosition(self.j2)
        j3_r = self.sim.getJointPosition(self.j3)
        j4_r = self.sim.getJointPosition(self.j4)
        j5_r = self.sim.getJointPosition(self.j5)
        j6_r = self.sim.getJointPosition(self.j6)

        # Jacobian matrix
        J = Jacobian.jacobian(j1_r, j2_r, j3_r, j4_r, j5_r, j6_r)
        
        # J_ = R @ J
        j_J = J @ J.T
        # j_J = j_J / np.linalg.norm(j_J)
        # epsilon = 1e-6  # Small regularization term
        # j_J = j_J + epsilon * np.eye(j_J.shape[0])
        # print(f"j_J의 최소값: {j_J.min()}, 최대값: {j_J.max()}")
        # if np.isnan(j_J).any() or np.isinf(j_J).any():
        #     print("행렬에 NaN 또는 Inf 값이 포함되어 있습니다.")
        #     print(j_J)
        #     j_J = np.nan_to_num(j_J, nan=1e-10, posinf=1e10, neginf=-1e10)
        # cond = np.linalg.cond(j_J)
        # print(f"Condition number: {cond}")
        # if cond > 1e10:  # 조건수가 너무 큰 경우
        #     print("Matrix is ill-conditioned.")
        # max_val = np.max(np.abs(j_J))
        # print(f"Max value in j_J: {max_val}")
        U, Sigma, Vt = scipy.linalg.svd(j_J, full_matrices=True, lapack_driver='gesvd')
        
        # SVD 계산
        try:
            U, Sigma, Vt = scipy.linalg.svd(j_J, full_matrices=True)
            self.U_previous, self.Sigma_previous, self.Vt_previous = U, Sigma, Vt  # 이전 값 업데이트
        except ValueError as e:
            print("SVD computation failed due to invalid input:")
            print(e)
            U, Sigma, Vt = self.U_previous, self.Sigma_previous, self.Vt_previous  # 이전 값 사용

        
        X_d = self.sim.getObjectPosition(self.dummy, -1)
        X_d = np.array(X_d)
        X_c = self.sim.getObjectPosition(self.Ur5_EE, -1)
        X_c = np.array(X_c)

        
        X_d_list.append(X_d)

        if len(X_d_list) > 9:
            # print(f"==>> len(X_d_list): {len(X_d_list)}")
            for i, z in enumerate(X_d_list):
                z = np.array(z).reshape(-1, 1)  # Convert to column vector
                # Prediction step
                control_input = np.array([0, 0, 0]).reshape(-1, 1)  # No control input
                predicted_state, predicted_covariance = Kalman.kalman_filter_predict(self.state, covariance, A, B, control_input, Q)
                # Update step
                self.state, covariance = Kalman.kalman_filter_update(predicted_state, predicted_covariance, H, z, R)
            future_time_step = 3.0  # Time steps to predict into the future
            A_future = A.copy()
            # print(f"==>> A_future: {A_future}")
            A_future[0:3, 3:6] *= future_time_step  # Adjust for future velocity scaling
            predicted_future_state, _ = Kalman.kalman_filter_predict(self.state, covariance, A_future, B, control_input, Q)
            # print(f"==>> predicted_future_state: {predicted_future_state}")
            X_d = predicted_future_state[0:3].flatten()
            X_d_list.pop(0)

        else:
            
            state = np.array([X_d[0], X_d[1], X_d[2],(X_d[0]-self.state[3][0])/dt, (X_d[1]-self.state[4][0])/dt, (X_d[2]-self.state[5][0])/dt ]).reshape(-1, 1)

        d_goal = X_d - X_c
        d_goal_2D = X_d[:2] - X_c[:2]
        d_goal_unit = d_goal/np.linalg.norm(d_goal)
        d_goal_unit_2D = d_goal_2D/np.linalg.norm(d_goal_2D)
        manipulability_direction = U[:, 0]
        manipulability_direction_array = np.array(manipulability_direction)


        # 시작점 (엔드 이펙터 위치)
        start_point = X_c

        # 벡터 끝점 정의 (스케일 조절 가능)
        scale = 0.2  # 화살표 길이를 조정하는 스케일링 값
        end_point_ = start_point + scale * d_goal_unit
        
        # 화살표 생성 (start_point에서 end_point로)
        line_handle = self.sim.addDrawingObject(
                self.sim.drawing_lines,  # objectType: 선 그리기
                0.01,               # size: 선 두께
                0.0,                # duplicateTolerance: 중복 허용 (0으로 비활성화)
                -1,                 # parentHandle: 월드 좌표계
                2,                  # maxItemCount: 최대 2개의 점 (시작점과 끝점)
                [1, 0, 0]           # 색상: 빨간색 (RGB)
            )
        line_handle_ = self.sim.addDrawingObject(
                self.sim.drawing_lines,  # objectType: 선 그리기
                0.01,               # size: 선 두께
                0.0,                # duplicateTolerance: 중복 허용 (0으로 비활성화)
                -1,                 # parentHandle: 월드 좌표계
                2,                  # maxItemCount: 최대 2개의 점 (시작점과 끝점)
                [0, 1, 0]           # 색상: 빨간색 (RGB)
            )
        # self.sim.addDrawingObjectItem(line_handle, list(start_point) + list(end_point))
        self.sim.addDrawingObjectItem(line_handle_, list(start_point) + list(end_point_))

        time_interval = 0.05
        angle_con_value = 1.0
        # 최대 속도 한계 설정 (예시)
        max_q_dot = 0.1 # 최대 속도 한계를 설정
        print('distance: ', np.linalg.norm(d_goal))
        if np.linalg.norm(d_goal) < 0.01:
            # self.previous_time = t 
            # 최대 힘 제한을 설정
            self.sim.setJointForce(self.j1_h, self.maxForce)
            self.sim.setJointForce(self.j2_h, self.maxForce)
            self.sim.setJointForce(self.j3_h, self.maxForce)
            self.sim.setJointForce(self.j4_h, self.maxForce)
            self.sim.setJointForce(self.j5_h, self.maxForce)
            self.sim.setJointForce(self.j6_h, self.maxForce)
            l_v = 0.0
            a_v = 0.0
            w_R, w_L, _ = self.set_velocity(l_v, a_v, 0, 0, 0, 0, 0, 0, coppel_controller)

        elif np.linalg.norm(d_goal) > 0.25:
            # self.previous_time = t 
            l_v, a_v = coppel_controller.kinematic_control(self.integral_dist, self.previous_err_dist,self.integral_theta, self.previous_err_theta, d_goal_2D[0]*10, d_goal_2D[1]*10) #cm

            vectorsize = 0.002
            new_d_goal = d_goal_unit * vectorsize
            d_goal_v = new_d_goal/(dt)
            epsilon = 1e-6  # 작은 특이값에 대한 임계값
            Sigma_inv = np.diag(1.0 / (Sigma + epsilon))  # 작은 특이값에 임계값을 더하여 안정화
            # pseudo inverse jacobian matrix
            J_pseudo = Vt.T @ Sigma_inv @ U.T
            
            
            q_dot = J_pseudo @ d_goal_v

            # 속도가 한계를 초과하면 제한
            q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)

            j1_tv = q_dot[0]
            j2_tv = q_dot[1]
            j3_tv = q_dot[2]

            # j4_tv = q_dot[3]
            # j5_tv = q_dot[4]
            # j6_tv = q_dot[5]
            w_R, w_L, wc = self.set_velocity(l_v, a_v, j1_tv, j2_tv, j3_tv, 0, 0, 0, coppel_controller)

            self.sim.addDrawingObjectItem(line_handle, None)
            self.sim.addDrawingObjectItem(line_handle_, None)

        else:
            # self.previous_time = t 
            l_v, a_v = coppel_controller.kinematic_control(self.integral_dist, self.previous_err_dist,self.integral_theta, self.previous_err_theta, d_goal_2D[0]*10, d_goal_2D[1]*10) #cm

            new_d_goal = d_goal_unit #* vectorsize
            d_goal_v = new_d_goal/(dt)
            epsilon = 1e-6  # 작은 특이값에 대한 임계값
            Sigma_inv = np.diag(1.0 / (Sigma + epsilon))  # 작은 특이값에 임계값을 더하여 안정화
            # pseudo inverse jacobian matrix
            J_pseudo = Vt.T @ Sigma_inv @ U.T
            
            
            q_dot = J_pseudo @ d_goal_v

            # 속도가 한계를 초과하면 제한
            q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
            j1_tv = q_dot[0]
            j2_tv = q_dot[1]
            j3_tv = q_dot[2]

            # j4_tv = q_dot[3]
            # j5_tv = q_dot[4]
            # j6_tv = q_dot[5]
            w_R, w_L, wc = self.set_velocity(l_v, a_v, j1_tv, j2_tv, j3_tv, 0, 0, 0, coppel_controller)

            self.sim.addDrawingObjectItem(line_handle, None)
            self.sim.addDrawingObjectItem(line_handle_, None)
        self.moveToAngle_RL(w_R, w_L)

    def ft_process(self, msg):
        t = self.sim.getSimulationTime()
        dt = t - self.previous_time
        if dt <= 0 or np.isnan(dt):
            print("dt가 0이거나 NaN입니다. 기본값으로 설정합니다.")
            dt = 1e-6  # 매우 작은 기본값
        self.previous_time = t
        # print(f"==>> self.previous_time: {self.previous_time}")
        self.coppeliasim(self.Kalman, self.Jacobian, dt, self.X_d_list, self.coppel_simulator)
        self.client.step()
        # Initialize controller
        admittance_controller = AdmittanceController(self.M, self.B, self.K)
        ft = []
        ft.append(msg.wrench.force.x)
        ft.append(msg.wrench.force.y)
        ft.append(msg.wrench.force.z)
        ft.append(msg.wrench.torque.x)
        ft.append(msg.wrench.torque.y)
        ft.append(msg.wrench.torque.z)
        print('ft', ft)
        force_torque_input = np.array([ft[0], ft[1], ft[2], ft[3], ft[4], ft[5]]) # [Fx, Fy, Fz, Tx, Ty, Tz]
        delta_position = admittance_controller.update(force_torque_input, dt)
        if delta_position[0]<0.001:
            delta_position[0]=0
        if delta_position[1]<0.001:
            delta_position[1]=0
        if delta_position[2]<0.001:
            delta_position[2]=0
        print(f"==>> position: {delta_position}")
        

def main(args=None):
    rclpy.init(args=args)
    listener = Coppel_Ros2()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
if __name__ == '__main__':
    main()
    






