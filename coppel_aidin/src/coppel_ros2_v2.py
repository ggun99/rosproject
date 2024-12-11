from cv2 import threshold
import rclpy
from rclpy.node import Node 
import sys
import os
import time
import numpy as np
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
from coppel_jaco_kalman_class_moving import Coppeliasim

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
       self.coppel = Coppeliasim(self.sim)
       self.client.step()
       self.sim.startSimulation()
       self.previous_time = 0
       self.X_d_list = []



    def ft_process(self, msg):
        t = self.sim.getSimulationTime()
        dt = t - self.previous_time
        if dt <= 0 or np.isnan(dt):
            print("dt가 0이거나 NaN입니다. 기본값으로 설정합니다.")
            dt = 1e-6  # 매우 작은 기본값
        self.previous_time = t

        # Initialize controller
        admittance_controller = AdmittanceController(self.M, self.B, self.K)
        ft = []
        ft.append(msg.wrench.force.x)
        ft.append(msg.wrench.force.y)
        ft.append(msg.wrench.force.z)
        ft.append(msg.wrench.torque.x)
        ft.append(msg.wrench.torque.y)
        ft.append(msg.wrench.torque.z)
        threshold = 0.6
        mean_x = -5
        mean_y = 2.5
        mean_z = -9
        if np.abs(ft[0]-mean_x) < threshold:
            ft[0] = 0
        if np.abs(ft[1]-mean_y) < threshold:
            ft[1] = 0
        if np.abs(ft[2]-mean_z) < threshold:
            ft[2] = 0
        ft[0] = ft[0]-mean_x
        ft[1] = ft[1]-mean_y
        ft[2] = ft[2]-mean_z
        # print('ft', ft)
        force_torque_input = np.array([ft[0], ft[1], ft[2], ft[3], ft[4], ft[5]]) # [Fx, Fy, Fz, Tx, Ty, Tz]
        delta_position = admittance_controller.update(force_torque_input, dt)
        # print(f"==>> delta_position: {delta_position}")
        if np.abs(delta_position[0])<0.001:
            delta_position[0]=0
        if np.abs(delta_position[1])<0.001:
            delta_position[1]=0
        if np.abs(delta_position[2])<0.001:
            delta_position[2]=0
        # delta_position = list(map(lambda x: x * 0.01, delta_position))
        # print(f"==>> position: {delta_position}")
        self.coppel.coppeliasim(self.Kalman, self.Jacobian, dt, self.X_d_list, self.coppel_simulator, delta_position)
        self.client.step()
        

def main(args=None):
    rclpy.init(args=args)
    listener = Coppel_Ros2()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
if __name__ == '__main__':
    main()
    






