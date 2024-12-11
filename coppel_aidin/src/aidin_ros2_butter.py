import usb
import can
import sys
import os
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, lfilter

wd = os.path.abspath(os.getcwd())
sys.path.append(str(wd))
sys.path.append("/home/airlab/Documents/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/programming/zmqRemoteApi/clients/python")


class AFT20D15(Node):
    def __init__(self, mode="usb"):
        super().__init__('aidin_ftsensor_node')
        self.forceid = int(0x01A)
        self.torqueid = int(0x01B)
        
        # 설정에 따른 CAN 버스 초기화
        if mode == "usb":
            self.dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
            self.bus = can.Bus(interface="gs_usb",
                               channel=self.dev.product,
                               bus=self.dev.bus,
                               address=self.dev.address,
                               bitrate=1000000)  # sensor is running on 1000kbps
        elif mode == "robotell":
            self.bus = can.Bus(interface="robotell",
                               channel="/dev/ttyUSB1@115200",
                               rtscts=True,
                               bitrate=1000000)
        elif mode == "socket":
            self.bus = can.Bus(channel="can0", interface="socketcan")

        # ROS2 Publisher 초기화
        self.ftPub = self.create_publisher(WrenchStamped, '/aidin_ftsensor', 10)

        # 이전 데이터 저장
        self.U_previous = None
        self.Sigma_previous = None
        self.Vt_previous = None
        
        # ft data list 저장
        self.ft_x = []
        self.ft_y = []
        self.ft_z = []
        # Timer 설정
        self.timer = self.create_timer(0.01, self.publish_ft_data)  # 10ms 간격

    def byte_to_output(self, bytearray):
        intar = list(bytearray)
        xout = intar[0] * 256 + intar[1]
        yout = intar[2] * 256 + intar[3]
        zout = intar[4] * 256 + intar[5]
        return [xout, yout, zout]

    def get_force(self, data):
        return [data[0] / 1000.0 - 30.0,
                data[1] / 1000.0 - 30.0,
                data[2] / 1000.0 - 30.0]

    def get_torque(self, data):
        return [data[0] / 100000.0 - 0.3,
                data[1] / 100000.0 - 0.3,
                data[2] / 100000.0 - 0.3]

    def receive(self):
        ft = [None] * 6
        for _ in range(2):
            rxmsg: can.Message = self.bus.recv(timeout=1)
            if rxmsg is not None:
                databytes = list(rxmsg.data)
                dataints = self.byte_to_output(databytes)
                canid = rxmsg.arbitration_id
                if canid == self.forceid:
                    force = self.get_force(dataints)
                    ft[0] = force[0]
                    ft[1] = force[1]
                    ft[2] = force[2]
                elif canid == self.torqueid:
                    torque = self.get_torque(dataints)
                    ft[3] = torque[0]
                    ft[4] = torque[1]
                    ft[5] = torque[2]
        return ft
           # Butterworth 저역통과 필터 설계
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

    def publish_ft_data(self):
        ft = self.receive()
        if ft:
            # Filter requirements.
            cutoff = 5.0  # 저역통과 필터의 컷오프 주파수
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
            # WrenchStamped 메시지 생성
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.header.frame_id = "sensor_frame"
            # Force 데이터 설정
            wrench_msg.wrench.force.x = ft[0] if ft[0] else 0.0
            wrench_msg.wrench.force.y = ft[1] if ft[1] else 0.0
            wrench_msg.wrench.force.z = ft[2] if ft[2] else 0.0

            # Torque 데이터 설정
            wrench_msg.wrench.torque.x = ft[3] if ft[3] else 0.0
            wrench_msg.wrench.torque.y = ft[4] if ft[4] else 0.0
            wrench_msg.wrench.torque.z = ft[5] if ft[5] else 0.0

            # 데이터 발행
            self.ftPub.publish(wrench_msg)

    def shutdown(self):
        self.bus.shutdown()
        self.get_logger().info("Sensor shutdown")


def main(args=None):
    rclpy.init(args=args)
    sensor_node = AFT20D15(mode="robotell")
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        sensor_node.shutdown()
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()





