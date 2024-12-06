import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os
import numpy as np
import pyrealsense2 as rs2
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Point

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener(Node):
    def __init__(self, depth_image_topic, depth_info_topic):
        super().__init__('listener')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageDepthCallback, 10)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = self.create_subscription(msg_Image, confidence_topic, self.confidenceCallback, 1)
        self.intrinsics = None
        self.pix = [0,0]
        self.pix_grade = None
        self.subscription = self.create_subscription(Int64MultiArray, '/ball_center',self.sub_callback, 10)
        self.position_publisher = self.create_publisher(Int64MultiArray, '/ball_position', 10)
        self.publisher_ = self.create_publisher(Odometry, '/pose', 10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        
    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (self.pix[0], self.pix[1])
            self.pix = pix
            line = 'Depth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
    
            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                position = Int64MultiArray()
                position.data = [int(result[0]), int(result[1]), int(result[2])]
                self.position_publisher.publish(position)

                c = open('polar_coordinate_trajectory_predict.txt', 'a')
                c.write('{} {} {}\n'.format(result[0], result[1], result[2]))
                c.close()
                
                pose = Odometry()
                pose.header.frame_id = 'camera_link' 
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position = Point(x=float(result[1]/100), y = float(result[0]/100), z = float(result[2]/100))  # x 좌표 설정
                # pose.pose.position.y = float(result[1])  # y 좌표 설정
                # pose.pose.position.z = float(result[2] - 200)  # z 좌표 설정
                self.publisher_.publish(pose)
                # log = 'Current time: %s' % self.get_clock().now().to_msg()
                # self.get_logger().info(log)

                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return
        
    def sub_callback(self, msg):
        self.pix = msg.data



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

def main(args=None):
    rclpy.init(args=args)
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'

    print ()
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ()
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ()
    listener = ImageListener(depth_image_topic, depth_info_topic)
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()