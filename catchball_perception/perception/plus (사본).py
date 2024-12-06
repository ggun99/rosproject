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
import serial
from time import time 
from scipy.optimize import curve_fit

global center
center = [0, 0]


if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener(Node):
    def __init__(self, depth_image_topic, depth_info_topic):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            msg_Image, 
            '/camera/color/image_raw', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
        self.getballposition
        #self.center_publisher = self.create_publisher(Int64MultiArray, '/ball_center', 10)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageDepthCallback, 10)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = self.create_subscription(msg_Image, confidence_topic, self.confidenceCallback, 1)
        self.intrinsics = None
        self.pix = [0,0]
        self.pix_grade = None
        #self.subscription = self.create_subscription(Int64MultiArray, '/ball_center',self.sub_callback, 10)
        self.position_publisher = self.create_publisher(Int64MultiArray, '/ball_position', 30)
        #self.publisher_ = self.create_publisher(Odometry, '/pose', 10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def getballposition(self, filename):
        img = filename
        
        #t1 = time()
        # define kernel size
        kernel = np.ones((7, 7), np.uint8)

        # convert to hsv colorspace
        #cv2.imshow("real_original", img)
        hsv1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        #cv2.imshow("camera_original",hsv1)

        hsv2 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        #t2 = time()


        #cv2.imshow("camera_hsv", hsv2)
        # lower bound and upper bound for Yellow color
        lower_bound = np.array([20, 80, 80])
        upper_bound = np.array([30, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv2, lower_bound, upper_bound)
        #t3 = time()
        #cv2.imshow("mask",mask)

        # define kernel size
        kernel = np.ones((7, 7), np.uint8)

        # Remove unnecessary noise from mask

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Segment only the detected region
        segmented_img = cv2.bitwise_and(img, img, mask=mask)
        #t4 = time()

        #gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        gray = cv2.medianBlur(mask, 9)

        #cv2.imshow("blur",gray)

        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=10,
                               minRadius=5, maxRadius=100)

        #t5 = time()

        if circles is not None:
            
            circles = np.uint16(np.around(circles))
            circles2=sorted(circles[0],key=lambda x:x[2],reverse=True)
            
            
            i = circles2[0]
            if (i[0] >= gray.shape[0]) or (i[1] >= gray.shape[1]) :
                return mask
            
            global center
            center = (i[0], i[1])
            #print(center)
            #msg = Int64MultiArray()
            #msg.data = [int(i[0]), int(i[1])]
            #self.center_publisher.publish(msg)
            # circle center
            #cv2.circle(mask, center, 1, (0, 100, 100), 3)

            cv2.circle(hsv1, center, 1, (255,0,0), 3)
            
            cv2.imshow("test",hsv1)
            #self.center_publisher = self.create_publisher(msg/Image, '/ball_center', 10)
            #imgname = str(self.imgnum)
            #path = os.getcwd()
            #print(path)
            #cv2.imwrite(imgname+".png", hsv1)
            #self.imgnum = self.imgnum + 1
        #t6 =time()
        # print("2d:", t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, t6-t1)
        return mask
    
    def imageDepthCallback(self, data):
        try:
            global center
            #t1 = time()
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (center[0], center[1])
            self.pix = pix
            # print(pix[0], pix[1])
            #line = 'Depth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[0], pix[1]])
    
            if self.intrinsics:
                depth = cv_image[pix[0], pix[1]]
                print(depth.shape)
                if depth > 3000:
                    return pix    
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                
                def rotx(theta):
                    R = np.array([[1,             0,              0],
                                  [0, np.cos(theta), -np.sin(theta)],
                                  [0, np.sin(theta),  np.cos(theta)]])
                    return R
                def roty(theta):
                    R = np.array([[np.cos(theta),  0,  np.sin(theta)],
                                  [            0,  1,              0],
                                  [-np.sin(theta), 0,  np.cos(theta)]])
                    return R
                def rotz(theta):
                    R = np.array([[np.cos(theta), -np.sin(theta),  0],
                                  [np.sin(theta),  np.cos(theta),  0],
                                  [            0,              0,  1]])
                    return R
                
                # dataDeque = deque()
                # for i in range(0,10):
                #     dataDeque.append(result)
                
                parray = enqueue(qlist, result)
                ppp = parray.T
                hx = rotx(-np.pi / 2)
                hz = rotz(np.pi / 2)

                pnew = hz @ hx @ ppp
                pnew = pnew.T

                x = pnew[:, 0]
                y = pnew[:, 1]
                z = pnew[:, 2]
                t = np.linspace(0,1/3,10)

                def parabolic(x, a, b, c):
                    return a * x**2 + b * x + c
                def deparabolic(z2, a, b, c):
                    return (-b+np.roots(b**2-4*a*(c-z2)))/2*a

                poptX, pcovX = curve_fit(parabolic, t, x)
                poptY, pcovY = curve_fit(parabolic, t, y)
                poptZ, pcovZ = curve_fit(parabolic, t, z)

                tReq = deparabolic(300,*poptZ)
                
                xPred = parabolic(tReq, *poptX)
                yPred = parabolic(tReq, *poptY)
                #zPred = parabolic(tReq, *poptZ)

                pred_position = np.array(xPred,yPred,300)

                position = Int64MultiArray()
                #position.data = [int(result[0]), int(result[1]), int(result[2])]
                position.data = [int(pred_position[0]), int(pred_position[1]), int(pred_position[2])]
                self.position_publisher.publish(position)
                


                # c = open('polar_coordinate_trajectory_predict.txt', 'a')
                # c.write('{} {} {}\n'.format(result[0], result[1], result[2]))
                # c.close()
                
                # pose = Odometry()
                # pose.header.frame_id = 'camera_link' 
                # pose.header.stamp = self.get_clock().now().to_msg()
                # pose.pose.pose.position = Point(x=float(result[1]/100), y = float(result[0]/100), z = float(result[2]/100))  # x 좌표 설정
                ## pose.pose.position.y = float(result[1])  # y 좌표 설정
                ## pose.pose.position.z = float(result[2] - 200)  # z 좌표 설정
                #self.publisher_.publish(pose)
                ## log = 'Current time: %s' % self.get_clock().now().to_msg()
                ## self.get_logger().info(log)
                #print(position.data)
                #line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()
            #t2 = time()
            #print("3d:", t2-t1)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        
    def listener_callback(self, data):
        """
        Callback function.
        """
        
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')
 
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        frame = self.getballposition(current_frame)

    
        #cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))
        # Display image
        #cv2.imshow("camera",frame)
    
        cv2.waitKey(1)

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