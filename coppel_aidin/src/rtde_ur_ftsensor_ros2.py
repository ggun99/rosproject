import rtde_control
import rtde_receive
import rclpy


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


class UR5e_ftsensor(Node):
    def __init__(self):
        super().__init__('UR5e_ftsensor_node')
        self.ROBOT_IP = '192.168.0.3'  # 로봇의 IP 주소로 바꿔주세요
        # RTDE 수신 객체 생성
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        # ROS2 Publisher 초기화
        self.ftPub = self.create_publisher(WrenchStamped, '/ur_ftsensor', 10)
        
        # ft data list 저장
        self.ft_x = []
        self.ft_y = []
        self.ft_z = []
        # Timer 설정
        self.timer = self.create_timer(0.01, self.publish_ft_data)  # 10ms 간격

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
        # 힘/토크 센서 값 읽기
        wrench = self.rtde_r.getActualTCPForce()   # 중력 혹은 다른 힘들이 보정이 된 TCP 에서 측정된 힘
        # raw_wrench = self.rtde_r.getFtRawWrench()   # 중력 혹은 다른 힘들이 일체 보정이 되지 않은 raw 데이터
        # 6D 벡터: [Fx, Fy, Fz, Tx, Ty, Tz]
        print("Force/Torque values:", wrench)
        # print("Raw Force/Torque values:", raw_wrench)
        ft = wrench
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


def main(args=None):
    rclpy.init(args=args)
    sensor_node = UR5e_ftsensor()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



