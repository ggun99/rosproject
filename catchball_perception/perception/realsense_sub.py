import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from std_msgs.msg import Int64MultiArray
import os
from time import time
import math

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    #self.imgnum = 0
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/color/image_raw', 
      self.listener_callback, 
      10)
    #self.subscription # prevent unused variable warning
    self.getballposition
    self.center = None
    self.center_publisher = self.create_publisher(Int64MultiArray, '/ball_center', 10)
      
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

  def getballposition(self, filename):
    img = filename
    print(img.shape)
    t1 = time()
  
    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # convert to hsv colorspace
    hsv1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #print(hsv1.shape)
    # if hsv1.shape[0]>200:
    #   dst1 = cv2.resize(hsv1, dsize=(0, 0), fx=0.4, fy=0.3, interpolation=cv2.INTER_LINEAR)
    # cv2.imshow("camera_original",hsv1)

    hsv2 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


      # lower bound and upper bound for Yellow color
    lower_bound = np.array([20, 80, 80])
    upper_bound = np.array([30, 255, 255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv2, lower_bound, upper_bound)
    #mask = cv2.resize(mask, dsize=(0, 0), fx=0.4, fy=0.3, interpolation=cv2.INTER_LINEAR)
    #cv2.imshow("mask",mask)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # Remove unnecessary noise from mask

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Segment only the detected region
    #segmented_img = cv2.bitwise_and(img, img, mask=mask)


    #gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    gray = cv2.medianBlur(mask, 9)
    # if gray.shape[0]>200: 
    #   gray = cv2.resize(gray, dsize=(0, 0), fx=0.4, fy=0.3, interpolation=cv2.INTER_LINEAR)
    #cv2.imshow("blur",dst)
    #cv2.imshow("blur",gray)


    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 3,
                               param1=100, param2=10,
                               minRadius=8, maxRadius=70)

    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            self.center = (i[0], i[1])
            r = (i[2])
            # a1, b1 = center[0], center[1]
            # a2, b2 = center(i+1[0]), center(i+1[1])
            # dist = math.sqrt((a2-a1)**2+(b2-b1)**2)
            # if dist<2*r:
               
            print(self.center)
            msg = Int64MultiArray()
            msg.data = [int(i[0]), int(i[1])]
            self.center_publisher.publish(msg)
            # circle center
            #cv2.circle(mask, center, 1, (0, 100, 100), 3)

            cv2.circle(hsv1, self.center, 1, (255,0,0), 3)
            (x,y), (w,h) = self.center, (6*r,6*r)
            cv2.rectangle(hsv1, (x-3*r,y-3*r,w,h), 255, 1)
            
            ##cv2.imshow("test",hsv1)
            
              
            #self.center_publisher = self.create_publisher(msg/Image, '/ball_center', 10)
            #imgname = str(self.imgnum)
            #path = os.getcwd()
            #print(path)
            #cv2.imwrite(imgname+".png", hsv1)
            #self.imgnum = self.imgnum + 1
        print(time()-t1)    
        c = open('center_with_time.txt', 'a')
        c.write('{} {} {}\n'.format(self.center[0],self.center[1], time()-t1))
        c.close()
    else:
      self.center = None

    return mask

  # def move_rec(self,data):
  #    roi = data
  #    cent = (data[0],data[1])
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console``
    self.get_logger().info('Receiving video frame')
    
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    if self.center is not None:
      c = self.center[1]-80
      d = self.center[1]+80
      e = self.center[0]-80
      f = self.center[0]+80
      if c < 0:
          c = 0
      if d < 0:
          d = 0
      if e < 0:
          e = 0
      if f < 0:
          f = 0
      current_frame = current_frame[c:d,e:f]
    else:
      current_frame = current_frame
    ##cv2.imshow("c", current_frame)
    frame = self.getballposition(current_frame)
    
    #cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))
    # Display image
    #cv2.imshow("camera",frame)
    
    cv2.waitKey(1)


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()