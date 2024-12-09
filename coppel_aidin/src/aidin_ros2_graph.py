import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import pyqtgraph as pg
import PySide2
from PySide2 import QtWidgets, QtCore
import threading
import time
import usb.core
import can


class AFT20D15(Node):
    def __init__(self, mode):
        super().__init__('aft20d15_node')
        self.initialize_bus(mode)
        self.forceid = int(0x01A)
        self.torqueid = int(0x01B)
        self.publisher = self.create_publisher(WrenchStamped, '/aidin_ftsensor', 10)
        self.graph_initialized = False

    def initialize_bus(self, mode):
        if mode == "usb":
            self.dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
            self.bus = can.Bus(interface="gs_usb",
                               channel=self.dev.product,
                               bus=self.dev.bus,
                               address=self.dev.address,
                               bitrate=1000000)  # Sensor is running on 1000kbps
        elif mode == "robotell":
            self.bus = can.Bus(interface="robotell",
                               channel="/dev/ttyUSB1@115200",
                               rtscts=True,
                               bitrate=1000000)
        elif mode == "socket":
            self.bus = can.Bus(channel="can0", interface="socketcan")

    def byte_to_output(self, bytearray):
        intar = list(bytearray)
        xout = intar[0] * 256 + intar[1]
        yout = intar[2] * 256 + intar[3]
        zout = intar[4] * 256 + intar[5]
        return [xout, yout, zout]

    def get_force(self, data: list):
        return [data[0] / 1000.0 - 30.0,
                data[1] / 1000.0 - 30.0,
                data[2] / 1000.0 - 30.0]

    def get_torque(self, data: list):
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
                if canid == self.torqueid:
                    torque = self.get_torque(dataints)
                    ft[3] = torque[0]
                    ft[4] = torque[1]
                    ft[5] = torque[2]
        return ft

    def publish_data(self):
        try:
            while rclpy.ok():
                ft = self.receive()
                if all(value is not None for value in ft):
                    msg = WrenchStamped()
                    msg.wrench.force.x = ft[0]
                    msg.wrench.force.y = ft[1]
                    msg.wrench.force.z = ft[2]
                    msg.wrench.torque.x = ft[3]
                    msg.wrench.torque.y = ft[4]
                    msg.wrench.torque.z = ft[5]
                    self.publisher.publish(msg)
                time.sleep(0.001)  # Delay to prevent overloading the loop
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.bus.shutdown()

# PyQtGraph 업데이트 스레드
def start_graph(sensor: AFT20D15):
    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="Real-Time Plot")
    win.resize(800, 400)
    win.setWindowTitle('Real-Time Plot')

    plot = win.addPlot(title="Real-Time Data")
    plot.setLabel('left', 'Force/Torque')
    plot.setLabel('bottom', 'Time', 's')
    curve1 = plot.plot(pen='r', name="Force X")
    curve2 = plot.plot(pen='g', name="Force Y")
    curve3 = plot.plot(pen='b', name="Force Z")

    time_data = []
    force_data = [[], [], []]
    start_time = time.time()

    while True:
        ft = sensor.receive()
        cur_time = time.time() - start_time
        time_data.append(cur_time)

        if len(ft) >= 3:
            force_data[0].append(ft[0])
            force_data[1].append(ft[1])
            force_data[2].append(ft[2])

        curve1.setData(time_data, force_data[0])
        curve2.setData(time_data, force_data[1])
        curve3.setData(time_data, force_data[2])

        app.processEvents()
        time.sleep(0.001)  # 업데이트 속도

def main():
    rclpy.init()
    sensor_node = AFT20D15(mode="robotell")

    # ROS 2 노드와 PyQtGraph 스레드 실행
    graph_thread = threading.Thread(target=start_graph, args=(sensor_node,), daemon=True)
    graph_thread.start()

    try:
        # Start the publishing data loop in a separate thread
        publish_thread = threading.Thread(target=sensor_node.publish_data, daemon=True)
        publish_thread.start()

        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        print("Shutting down.")
        sensor_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
