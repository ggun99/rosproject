import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO
from time import time

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # t = time()
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/color/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.model = None
    self.model = YOLO("yolov8n.yaml")
    self.model = YOLO("yolov8n.pt")
    self.model.to('cuda')
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
    # t1 = time()-t 
    # print(t1)

  def listener_callback(self, data):
    """
    Callback function.
    """
    t1 = time()
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    #current_frame = cv2.resize(current_frame, dsize=(0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    #self.model.export(format='engine',device=0)
    #hsv1 = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    hsv1 = self.model.predict(current_frame, device = 0)
    #hsv1 = cv2.cvtColor(hsv1, cv2.COLOR_BGR2RGB)
    hsv1 = hsv1[0].plot()
    #cv2.imshow("test", hsv1)
    cv2.waitKey(1)
    t = time()-t1
    print(t)

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
