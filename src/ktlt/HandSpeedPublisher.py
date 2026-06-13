import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import HandTrackingModule as htm  

class HandSpeedNode(Node):
    def __init__(self):
        super().__init__('tank_drive_publisher')
        # Chuyển sang dùng topic /cmd_vel chuẩn cho Robot tự hành
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        # Tăng tốc Timer lên 30Hz để xe phản ứng tức thời
        self.timer = self.create_timer(0.033, self.timer_callback) 
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.detector = htm.handDetector(maxHands=2, detectionCon=0.7)
        
        # Thông số cơ khí của xe
        self.wheel_radius = 0.035  # Bán kính 35mm
        self.track_width = 0.20    # Khoảng cách 2 bánh 20cm

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            return

        # Lật ảnh để có hiệu ứng soi gương (Tay trái ở bên trái màn hình)
        img = cv2.flip(img, 1)
        
        img = self.detector.findHands(img, draw=True)
        
        rpm_L = 0.0
        rpm_R = 0.0

        # Nếu phát hiện có tay
        if self.detector.results.multi_hand_landmarks:
            num_hands = len(self.detector.results.multi_hand_landmarks)
            hands_data = []
            
            # Lấy tọa độ của tất cả các tay đang hiển thị
            for i in range(num_hands):
                lmList = self.detector.findPosition(img, handNo=i, draw=False)
                if lmList:
                    hands_data.append(lmList)
            
            # Sắp xếp các tay theo tọa độ X từ trái sang phải màn hình
            hands_data.sort(key=lambda hand: hand[0][1])

            for i, lmList in enumerate(hands_data):
                # 1. Tính độ mở bướm ga (ngón cái và ngón trỏ)
                x1, y1 = lmList[4][1], lmList[4][2]
                x2, y2 = lmList[8][1], lmList[8][2]
                length = math.hypot(x2 - x1, y2 - y1)
                
                # --- PHẦN VẼ ĐỒ HỌA DÂY THUN ---
                # Vẽ đường thẳng nối ngón cái và ngón trỏ (Màu vàng, nét 3)
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3) 
                
                # Vẽ thêm 2 chấm tròn ở chóp ngón để làm nổi bật (Màu hồng)
                cv2.circle(img, (x1, y1), 8, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 8, (255, 0, 255), cv2.FILLED)
                
                # Vẽ một chấm đỏ ở chính giữa đường thẳng
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
                # -------------------------------

                # Ép xung: Chỉ cần mở hờ 150 pixel là đạt max 150 RPM
                rpm = np.interp(length, [30, 150], [0, 150]) 
                
                # 2. Cần số ngón út (so sánh Y chóp ngón 20 và gốc 17)
                y20 = lmList[20][2]
                y17 = lmList[17][2]
                if y20 > y17:  # Ngón út gập xuống (tọa độ Y lớn hơn)
                    rpm = -rpm # Chạy lùi

                # Gán RPM cho bánh Trái hoặc Phải
                if len(hands_data) == 2:
                    if i == 0: 
                        rpm_L = rpm
                    else: 
                        rpm_R = rpm
                else:
                    # Nếu chỉ giơ 1 tay, cho 2 bánh chạy bằng nhau (đi thẳng)
                    rpm_L = rpm
                    rpm_R = rpm

                # Giao diện UI: Đánh dấu RPM từng tay
                text_x, text_y = lmList[0][1], lmList[0][2]
                cv2.putText(img, f'{int(rpm)}', (text_x, text_y + 30), 
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

        # 3. Động học thuận (Forward Kinematics)
        # Tính vận tốc (m/s) của từng bánh
        v_L = (rpm_L * 2 * math.pi / 60.0) * self.wheel_radius
        v_R = (rpm_R * 2 * math.pi / 60.0) * self.wheel_radius

        # Trộn thành vận tốc tuyến tính và vận tốc góc của xe
        v = (v_R + v_L) / 2.0
        w = (v_R - v_L) / self.track_width

        # 4. Đóng gói ROS 2 Twist Message
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)

        # In thông số ra màn hình để quan sát
        cv2.putText(img, f'V: {v:.2f} m/s | W: {w:.2f} rad/s', (20, 40), 
                    cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
                    
        cv2.imshow("Tank Drive Vision", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()