import rtde_control
import rtde_receive
import time

# UR 로봇의 IP 주소
ROBOT_IP = "192.168.0.212"  # 로봇의 실제 IP 주소로 변경

def check_connection(robot_ip):
    try:
        # RTDE Control 및 RTDE Receive 객체 생성
        rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        
        # 연결 확인
        if rtde_c.isConnected() and rtde_r.isConnected():
            print(f"성공적으로 연결되었습니다! 로봇 IP: {robot_ip}")
            
            # 현재 TCP 위치 출력
            tcp_pose = rtde_r.getActualTCPPose()
            print(f"현재 TCP 위치: {tcp_pose}")
            
            # 연결 종료
            rtde_c.disconnect()
            rtde_r.disconnect()
            print("연결을 종료했습니다.")
        else:
            print(f"로봇에 연결하지 못했습니다. IP: {robot_ip}")
    except Exception as e:
        print(f"연결 중 오류가 발생했습니다: {e}")

if __name__ == "__main__":
    check_connection(ROBOT_IP)