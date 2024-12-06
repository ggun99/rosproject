import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from rb_interfaces.srv import ReqTcp
from scipy.spatial.transform import Rotation as R

class Calculator(Node):
    def __init__(self):
        super().__init__('homogeneous_matrix_node')
        self.rb_pose_sub = self.create_subscription(PoseStamped, '/current_rb_pose',self.matrix_b2e_pose, 10)
        # self.rb_jonin_sub = self.create_subscription(JointState, '/current_rb_joint',self.matrix_b2e_joint, 10)
        self.poseSub = self.create_subscription(PoseArray, '/pose', self.cal_pose, 10)
        self.mat_b2e = np.array([[0.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.0]])
        
        self.mat = PoseStamped()
        self.ballposematpub = self.create_publisher(PoseStamped,'/mat',10)
        self.cli = self.create_client(ReqTcp,'req_tcp')
       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def matrix_b2e_pose(self, rb_pose):
        x_p = rb_pose.pose.position.x
        y_p = rb_pose.pose.position.y
        z_p = rb_pose.pose.position.z
        Rx_p_q = rb_pose.pose.orientation.x
        Ry_p_q = rb_pose.pose.orientation.y
        Rz_p_q = rb_pose.pose.orientation.z
        Rw_p_q = rb_pose.pose.orientation.w
        q = np.array([Rx_p_q, Ry_p_q, Rz_p_q, Rw_p_q])
        r = R.from_quat(q)
        rpy = r.as_euler('zyx', degrees=True)
        # print(f"==>> rpy: {rpy}")
        rpy_x = np.radians(rpy[0])
        rpy_y = np.radians(rpy[1])
        rpy_z = np.radians(rpy[2])

        rpy_R = np.array([[1.0,       0.0,        0.0           ],
                          [0.0,  np.cos(rpy_x), -np.sin(rpy_x)],
                          [0.0,  np.sin(rpy_x),  np.cos(rpy_x)]])@np.array([[ np.cos(rpy_y), 0.0,  np.sin(rpy_y)],
                                                                              [0.0,             1.0,      0.0       ],
                                                                              [-np.sin(rpy_y), 0.0,  np.cos(rpy_y)]])@np.array([[np.cos(rpy_z), -np.sin(rpy_z),0.0],
                                                                                                                                  [np.sin(rpy_z),  np.cos(rpy_z),0.0],
                                                                                                                                  [0.0,             0.0,           1.0]])
        # print(f"==>> rpy_R: {rpy_R}")
        self.mat_b2e = np.array([[rpy_R[0,0],rpy_R[0,1],rpy_R[0,2],x_p],
                                 [rpy_R[1,0],rpy_R[1,1],rpy_R[1,2],y_p],
                                 [rpy_R[2,0],rpy_R[2,1],rpy_R[2,2],z_p],
                                 [0.0,       0.0,       0.0,       1.0]])
        # print(f"==>> self.mat_b2e: {self.mat_b2e}")
        
    def cal_pose(self, msg):
        mat_e2c = np.array([[0.0,  1.0, 0.0,    30.],
                            [-1.0, 0.0, 0.0,   -27.],
                            [0.0,  0.0, 1.0,    62.],
                            [0.0,  0.0, 0.0,    1.0]])
        # mat_e2c = np.array([[0.0,  1.0, 0.0,    15.],
        #                     [-1.0, 0.0, 0.0,   -27.],
        #                     [0.0,  0.0, 1.0,    62.],
        #                     [0.0,  0.0, 0.0,    1.0]])
       
        for pose in enumerate(msg.poses): 
            # print('pose',pose)  
            x = pose[1].position.x
            y = pose[1].position.y
            z = pose[1].position.z
            # print('x,y,z',x,y,z)
           
        mat_c2p = np.array([[1.0,0.0,0.0,x],
                            [0.0,1.0,0.0,y],
                            [0.0,0.0,1.0,z],
                            [0.0,0.0,0.0,1.0]])
        # print(f"==>> mat_c2p: {mat_c2p}")
        
        mat_b2p = self.mat_b2e@mat_e2c@mat_c2p
        # print(f"==>> mat_b2p: {mat_b2p}")
        self.R_matrix = np.array([[mat_b2p[0,0],mat_b2p[0,1],mat_b2p[0,2]],
                                  [mat_b2p[1,0],mat_b2p[1,1],mat_b2p[1,2]],
                                  [mat_b2p[2,0],mat_b2p[2,1],mat_b2p[2,2]] ])
        # print('mat', self.R_matrix)
        r = R.from_matrix(self.R_matrix)
        rpy = r.as_quat()
        
        self.mat.pose.position.x = mat_b2p[0,3]
        self.mat.pose.position.y = mat_b2p[1,3]
        self.mat.pose.position.z = mat_b2p[2,3]
        self.mat.pose.orientation.x = rpy[0]
        self.mat.pose.orientation.y = rpy[1]
        self.mat.pose.orientation.z = rpy[2]
        self.mat.pose.orientation.w = rpy[3]
        self.ballposematpub.publish(self.mat)
        #print('final matrix', self.mat.pose)
        matrix = np.zeros((0,3))
        coordinate = np.array([self.mat.pose.position.x,self.mat.pose.position.y,self.mat.pose.position.z])     
        matrix = np.vstack([matrix, coordinate])
        
        print(f"[{matrix[0][0]},{matrix[0][1]},{matrix[0][2]}]")
        # if matrix[0,2] is not 0:
        #     print(matrix)
        # else:
        #     return

 
    def send_request(self):
        req = ReqTcp.Request()
        # r = R.from_euler('zyx', self.mat.pose.orientation, degrees=True)
        # q = r.as_quat()
        req.tcppose.pose.position.x = self.mat.pose.position.x
        req.tcppose.pose.position.y = self.mat.pose.position.y
        req.tcppose.pose.position.z = self.mat.pose.position.z
        req.tcppose.pose.orientation.x = self.mat.pose.orientation.x
        req.tcppose.pose.orientation.y = self.mat.pose.orientation.y
        req.tcppose.pose.orientation.z = self.mat.pose.orientation.z
        req.tcppose.pose.orientation.w = self.mat.pose.orientation.w
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    listener = Calculator()
    rclpy.spin(listener)
    # response = listener.send_request()
    # listener.get_logger().info(f"res {response.success}, {response.message}")
    # rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
if __name__ == '__main__':
    main()