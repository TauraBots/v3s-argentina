import socket
import struct
import math
import requests
import time
import wrapper_pb2 as wr


class FiraClient:
    def __init__(self, vision_ip="224.5.23.2", vision_port=10015,
                 esp_ip="192.168.53.145", esp_port=8080, robot_id=5):
        self.vision_ip = vision_ip
        self.vision_port = vision_port
        self.esp_ip = esp_ip
        self.esp_port = esp_port
        self.robot_id = robot_id
        self.last_command_time = time.time()

        self.vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.vision_sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            struct.pack("=4sl", socket.inet_aton(self.vision_ip), socket.INADDR_ANY)
        )
        self.vision_sock.bind((self.vision_ip, self.vision_port))

    def send_motor_commands(self, vl, vr, raio_roda=0.0325):
        rpm_l = (vl * 60) / (2 * math.pi * raio_roda)
        rpm_r = (vr * 60) / (2 * math.pi * raio_roda)

        url = f"http://{self.esp_ip}:{self.esp_port}/velocity"
        payload = {"vl": rpm_l, "vr": rpm_r}

        try:
            response = requests.post(url, data=payload, timeout=0.1)
            print(f"📤 Enviado para ESP32 → RPM_L={rpm_l:.2f}, RPM_R={rpm_r:.2f}")
            print(f"  → Status: {response.status_code}, Resposta: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"❌ Erro ao enviar comandos para ESP32: {e}")

    def angle_to_ball(self, robot_x, robot_y, ball_x, ball_y):
        dx = ball_x - robot_x
        dy = ball_y - robot_y
        return math.atan2(dy, dx)

    def gotoball(self, robot, ball, angle_threshold=0.1):
        angle_to_ball = self.angle_to_ball(robot.x, robot.y, ball.x, ball.y)
        angle_diff = angle_to_ball - robot.orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        Kp_ang = 0.5
        Kp_lin = 6.0
        max_speed = 1.0

        distance = math.hypot(ball.x - robot.x, ball.y - robot.y)

        if abs(angle_diff) > angle_threshold:
            v_linear = 0.0
        else:
            v_linear = min(Kp_lin * distance, max_speed)

        v_angular = max(min(Kp_ang * angle_diff, 0.1), -0.1)

        vl = v_linear - v_angular
        vr = v_linear + v_angular

        self.send_motor_commands(vl, vr)
        self.last_command_time = time.time()

        print(f"  → Angle to Ball: {math.degrees(angle_to_ball):.2f}°")
        print(f"  → Angle Diff   : {math.degrees(angle_diff):.2f}°")
        print(f"  → Comandos     : VL={vl:.2f}, VR={vr:.2f}")

    def stop_if_needed(self):
        if time.time() - self.last_command_time > 0.5:
            print("⏱️ Timeout sem comando → Enviando parada de segurança (RPM 0)")
            self.send_motor_commands(0, 0)
            self.last_command_time = time.time()

    def receive_frame(self):
        data = None
        while True:
            try:
                data, _ = self.vision_sock.recvfrom(8192)
                break
            except Exception as e:
                print(f"[Socket Error] {e}")
                continue

        if data:
            try:
                wrapper = wr.SSL_WrapperPacket()
                wrapper.ParseFromString(data)

                ball = None
                robot_encontrado = False

                if wrapper.HasField("detection"):
                    detection = wrapper.detection

                    if detection.balls:
                        ball = detection.balls[0]
                        print(f"Ball Position: x={ball.x:.2f}, y={ball.y:.2f}")
                    else:
                        print("⚠️ Bola não detectada → Enviando parada.")
                        self.send_motor_commands(0, 0)

                    for robot in detection.robots_blue:
                        print(f"Blue Robot ID {robot.robot_id}: x={robot.x:.2f}, y={robot.y:.2f}, orientation={math.degrees(robot.orientation):.2f}°")
                        if ball and robot.robot_id == self.robot_id:
                            robot_encontrado = True
                            self.gotoball(robot, ball)

                    if not robot_encontrado:
                        print(f"⚠️ Robô ID {self.robot_id} não encontrado → Enviando parada.")
                        self.send_motor_commands(0, 0)

                self.stop_if_needed()

                if wrapper.HasField("geometry") and wrapper.geometry.HasField("field"):
                    field = wrapper.geometry.field
                    print("[FIELD DIMENSIONS]")
                    print(f"  Length       : {field.field_length}")
                    print(f"  Width        : {field.field_width}")
                    print(f"  Goal Width   : {field.goal_width}")
                    print(f"  Goal Depth   : {field.goal_depth}")
                    print(f"  Boundary     : {field.boundary_width}")
                    print(f"  Penalty Area : {field.penalty_area_width} x {field.penalty_area_depth}")
            except Exception as e:
                print(f"[Protobuf Error] {e}")


if __name__ == "__main__":
    a = FiraClient()
    while True:
        a.receive_frame()
