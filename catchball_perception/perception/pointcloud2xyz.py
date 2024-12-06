import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.ndimage import binary_dilation, binary_erosion
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from time import time

class CropLocalization(Node):
    def __init__(self):
        super().__init__('crop_detection_and_pose_node')
        self.imageSub = self.create_subscription(Image, '/camera/color/image_raw', self.img_process, 10)
        self.pointcloudSub = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.get_point_cloud, 10)
        self.imagePub = self.create_publisher(Image, '/debugimg', 10)
        self.poseArrayPub = self.create_publisher(PoseArray, '/ball_pose', 10)
        # # self.poseArrayTimer = self.create_timer(0.03, self.publish_pose_array)

        self.br = CvBridge()
        self.center = [0,0]
        self.row = None
        self.col = None
        self.pointCloud = None
        self.imageDrawn = None

    def publish_pose_array(self, px, py):
        t1 = time()
        if self.pointCloud is not None:
            poseArrayMsg = PoseArray()
            poseArrayMsg.header = Header()
            poseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            poseArrayMsg.header.frame_id = 'camera_link'

            index = ((py-1)*424) + px
            xyzc = self.pointCloud[index]
            x = xyzc[0]
            y = xyzc[1]
            z = xyzc[2]
            pose = Pose()           
            
            pose.position.x = float(z)
            pose.position.y = float(-x)
            pose.position.z = float(-y)
            print(f"==>> pose: {pose}")
            poseArrayMsg.poses.append(pose)
            self.poseArrayPub.publish(poseArrayMsg)
        t2 = time()
        print('time', t2-t1)

    def get_point_cloud(self, msg):
        self.pointCloud = self.point_cloud2_to_array(msg)

    def point_cloud2_to_array(self, msg):
        point_cloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]
        return point_cloud

    def img_process(self, msg):
        imgBGR = self.br.imgmsg_to_cv2(msg)
        
        # define kernel size
        kernel = np.ones((7, 7), np.uint8)

        # convert to hsv colorspace
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_RGB2HSV)

        # lower bound and upper bound for Yellow color
        lower_bound = np.array([20, 80, 80])
        upper_bound = np.array([30, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(imgHSV, lower_bound, upper_bound)
        
        # define kernel size
        kernel = np.ones((7, 7), np.uint8)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        gray = cv2.medianBlur(mask, 9)
        
        rows = gray.shape[0]
        
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=100, param2=10, minRadius=8, maxRadius=70)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            circles2 = sorted(circles[0],key=lambda x:x[2],reverse=True)
            xcenter = circles2[0][0]
            ycenter = circles2[0][1]
            radius = circles2[0][2]
            if xcenter > gray.shape[1]: xcenter = gray.shape[1]
            if ycenter > gray.shape[0]: ycenter = gray.shape[0]
            cv2.circle(imgRGB, center=(xcenter, ycenter), radius=radius, color=(255,0,0), thickness=3)
            self.publish_pose_array(px=xcenter, py=ycenter)
            
        self.pub_img(imgRGB)
  
    def pub_img(self, img):
        imgBGR = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        imgData = self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8")
        self.imagePub.publish(imgData)


def main(args=None):
    rclpy.init(args=args)
    # try:
    #     node = CropLocalization()
    #     executor = MultiThreadedExecutor()
    #     executor.add_node(node)
    #     try:
    #         executor.spin()
    #     finally:
    #         executor.shutdown()
    #         node.destroy_node()
    # finally:
    #     rclpy.shutdown()
    listener = CropLocalization()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
if __name__ == '__main__':
    main()