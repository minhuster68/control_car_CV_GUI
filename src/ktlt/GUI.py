import sys
import cv2
import threading
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QFormLayout, QGroupBox, QLabel, 
                             QPushButton, QSlider, QLCDNumber, QProgressBar, QGridLayout)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal

try:
    import HandTrackingModule as htm
except ImportError:
    pass

# ============================================================================
# 1. LUỒNG XỬ LÝ CAMERA & AI DEEP LEARNING (CHẠY NỀN)
# ============================================================================
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage) 
    velocity_signal = pyqtSignal(float, float) 

    def __init__(self):
        super().__init__()
        self._run_flag = True
        try:
            self.detector = htm.handDetector(maxHands=2, detectionCon=0.7)
        except:
            self.detector = None
        self.wheel_radius = 0.033
        self.track_width = 0.194

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)

        while self._run_flag:
            success, img = cap.read()
            if not success:
                continue                #chong crash do hong cam

            # Kích thước khung hình thực tế
            h, w_img, ch = img.shape

            rpm_L = 0.0
            rpm_R = 0.0
            
            # Mảng lưu thông tin để vẽ chữ RPM sau khi lật ảnh
            text_overlay_list = []

            if self.detector:
                img = self.detector.findHands(img, draw=True)
                if self.detector.results.multi_hand_landmarks:
                    num_hands = len(self.detector.results.multi_hand_landmarks)
                    hands_data = []     
                    
                    for i in range(num_hands):
                        lmList = self.detector.findPosition(img, handNo=i, draw=False) #danh sach toa do 21 dot khop tren tay, cho ca 2 ban tay
                        if lmList:
                            # Tính toán tọa độ X sau khi lật gương ngay từ đầu để đồng bộ góc nhìn người dùng
                            flipped_x_id0 = w_img - lmList[0][1]
                            hands_data.append((flipped_x_id0, lmList))
                    
                    # SẮP XẾP TAY THEO THỨ TỰ TỪ TRÁI SANG PHẢI TRÊN MÀN HÌNH GƯƠNG (Góc nhìn thực tế của bạn)
                    hands_data.sort(key=lambda item: item[0])

                    for i, (flipped_x_base, lmList) in enumerate(hands_data):
                        x1, y1 = lmList[4][1], lmList[4][2]
                        x2, y2 = lmList[8][1], lmList[8][2]
                        length = math.hypot(x2 - x1, y2 - y1)
                        
                        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3) 
                        cv2.circle(img, (x1, y1), 8, (255, 0, 255), cv2.FILLED)
                        cv2.circle(img, (x2, y2), 8, (255, 0, 255), cv2.FILLED)

                        rpm = np.interp(length, [30, 150], [0, 150]) 
                        
                        if lmList[20][2] > lmList[17][2]: 
                            rpm = -rpm

                        # PHÂN CHIA ĐỘNG HỌC THEO VỊ TRÍ TAY TRÊN MÀN HÌNH MẮT NGƯỜI NHÌN
                        if len(hands_data) == 2:
                            if i == 0: 
                                rpm_L = rpm  # Tay nằm bên TRÁI màn hình gương -> Điều khiển bánh TRÁI xe
                            else: 
                                rpm_R = rpm  # Tay nằm bên PHẢI màn hình gương -> Điều khiển bánh PHẢI xe
                        else:
                            rpm_L = rpm_R = rpm

                        # Lưu vị trí đầu ngón trỏ đã lật gương (X) để chuẩn bị hiển thị text RPM xuôi dòng
                        flipped_x_finger = w_img - lmList[8][1]
                        text_overlay_list.append((flipped_x_finger, lmList[8][2], int(rpm)))

            # Tính toán động học xuôi cho xe từ dữ liệu RPM chuẩn
            v_L = (rpm_L * 2 * math.pi / 60.0) * self.wheel_radius
            v_R = (rpm_R * 2 * math.pi / 60.0) * self.wheel_radius
            v = (v_R + v_L) / 2.0
            w = (v_R - v_L) / self.track_width

            self.velocity_signal.emit(float(v), float(w))
            
            # --- LẬT ẢNH GƯƠNG ĐỂ HIỂN THỊ ---
            img = cv2.flip(img, 1)

            # Vẽ chữ V, W (Xuôi chiều)
            cv2.putText(img, f'V: {v:.2f} m/s | W: {w:.2f} rad/s', (20, 40), 
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2)

            # Vẽ chữ RPM (Đã được đồng bộ tọa độ không bị nhảy hay ngược đầu)
            for (pos_x, pos_y, val_rpm) in text_overlay_list:
                cv2.putText(img, f'RPM: {val_rpm}', (pos_x, pos_y - 20), 
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            qt_img = QImage(img_rgb.data, w_img, h, 3 * w_img, QImage.Format_RGB888)
            self.change_pixmap_signal.emit(qt_img)

        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()


# ============================================================================
# 2. GIAO DIỆN TRUNG TÂM ĐIỀU KHIỂN CHÍNH
# ============================================================================
class ProDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_mode = "MANUAL"
        self.manual_speed = 0.20
        self.angular_speed = 2.5 

        self.init_ros()
        self.init_ui()
        
        self.ai_thread = VideoThread()
        self.ai_thread.change_pixmap_signal.connect(self.update_camera_frame)
        self.ai_thread.velocity_signal.connect(self.process_ai_velocity)
        self.ai_thread.start()

    def init_ros(self):
        rclpy.init()
        self.node = Node('pro_station_node')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True).start()
        self.ros_spin_timer = QtCore.QTimer(self)
        self.ros_spin_timer.timeout.connect(self.spin_ros)
        self.ros_spin_timer.start(10)

    def spin_ros(self):
        # Xử lý các tác vụ mạng của ROS 2 liên tục mà không gây kẹt giao diện
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def init_ui(self):
        self.setWindowTitle("HUST Autonomous Control Station Pro")
        self.resize(1200, 650)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #11121a; } 
            QLabel { color: #e2e8f0; font-family: 'Segoe UI'; font-size: 13px; } 
            QGroupBox { color: #00ffcc; font-weight: bold; font-size: 13px; border: 1px solid #334155; margin-top: 12px; border-radius: 6px; } 
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; } 
            QPushButton { background-color: #1e293b; color: #00ffcc; border: 1px solid #475569; border-radius: 4px; font-weight: bold; padding: 12px; } 
            QPushButton:pressed { background-color: #334155; }
            QPushButton:disabled { background-color: #0f172a; color: #475569; border: 1px solid #1e293b; }
        """)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # ---------- BÊN TRÁI: THÔNG SỐ VÀ NÚT ĐIỀU KHIỂN ----------
        left_layout = QVBoxLayout()
        left_layout.setSpacing(12)

        header_layout = QHBoxLayout()
        logo_label = QLabel()
        pixmap = QPixmap("HUST.jpg") 
        if not pixmap.isNull():
            logo_label.setPixmap(pixmap.scaledToHeight(80, Qt.SmoothTransformation))
        
        title_label = QLabel()
        title_label.setText("<span style='font-size: 14px; font-weight: bold; color: #f59e0b;'>ĐẠI HỌC BÁCH KHOA HÀ NỘI<br>"
                            "TRUNG TÂM ĐIỀU KHIỂN XE TỰ HÀNH</span>")
        
        header_layout.addWidget(logo_label)
        header_layout.addSpacing(10)
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        left_layout.addLayout(header_layout)

        telemetry_group = QGroupBox("GIÁM SÁT THỜI GIAN THỰC")
        tele_layout = QFormLayout(telemetry_group)
        tele_layout.setContentsMargins(15, 15, 15, 15)
        tele_layout.setSpacing(10)
        
        self.speed_display = QLCDNumber()
        self.speed_display.setSegmentStyle(QLCDNumber.Flat)
        self.speed_display.setFixedSize(140, 40)
        self.speed_display.setStyleSheet("color: #00ffcc; background: #1e293b; border: 1px solid #475569; border-radius: 4px;")
        self.speed_display.display(self.manual_speed)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.battery_bar.setFixedSize(200, 20)

        self.current_mode_label = QLabel("CHẾ ĐỘ: ĐIỀU KHIỂN THỦ CÔNG (MANUAL - WASD)")
        self.current_mode_label.setStyleSheet("font-size: 13px; color: #eab308; font-weight: bold;")

        tele_layout.addRow("VẬN TỐC THIẾT LẬP:", self.speed_display)
        tele_layout.addRow("MỨC NĂNG LƯỢNG:", self.battery_bar)
        tele_layout.addRow("TRẠNG THÁI HỆ THỐNG:", self.current_mode_label)
        left_layout.addWidget(telemetry_group)

        control_group = QGroupBox("BẢNG ĐIỀU KHIỂN D-PAD")
        control_group_layout = QVBoxLayout(control_group)
        control_group_layout.setContentsMargins(15, 15, 15, 15)
        control_group_layout.setSpacing(15)

        master_layout = QHBoxLayout()
        master_title = QLabel("TỐC ĐỘ:")
        self.master_slider = QSlider(Qt.Horizontal)
        self.master_slider.setRange(5, 55)
        self.master_slider.setValue(20)
        self.master_value_label = QLabel(f"{self.manual_speed:.2f} m/s")
        self.master_value_label.setFixedWidth(60)

        master_layout.addWidget(master_title)
        master_layout.addWidget(self.master_slider)
        master_layout.addWidget(self.master_value_label)
        control_group_layout.addLayout(master_layout)

        self.btn_up = QPushButton("TIẾN (W)")
        self.btn_down = QPushButton("LÙI (S)")
        self.btn_left = QPushButton("TRÁI (A)")
        self.btn_right = QPushButton("PHẢI (D)")
        self.btn_stop = QPushButton("STOP (SPACE)")
        self.btn_stop.setStyleSheet("QPushButton { background-color: #b91c1c; color: white; border: none; }")

        dpad_layout = QGridLayout()
        dpad_layout.setSpacing(8)
        dpad_layout.addWidget(self.btn_up,    0, 1)
        dpad_layout.addWidget(self.btn_left,  1, 0)
        dpad_layout.addWidget(self.btn_stop,  1, 1) 
        dpad_layout.addWidget(self.btn_right, 1, 2)
        dpad_layout.addWidget(self.btn_down,  2, 1)
        control_group_layout.addLayout(dpad_layout)
        left_layout.addWidget(control_group)

        self.btn_toggle_ai = QPushButton("KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION)")
        self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #4f46e5; color: white; padding: 12px; font-weight: bold; font-size: 13px; border-radius: 5px; }")
        left_layout.addWidget(self.btn_toggle_ai)

        self.btn_reset = QPushButton("DỪNG KHẨN CẤP (EMERGENCY STOP)")
        self.btn_reset.setStyleSheet("QPushButton { background-color: #dc2626; color: white; padding: 10px; font-weight: bold; border: none; }")
        left_layout.addWidget(self.btn_reset)

        main_layout.addLayout(left_layout, 1)

        # ---------- BÊN PHẢI: MÀN HÌNH CAMERA STREAM ----------
        right_layout = QVBoxLayout()
        self.camera_group = QGroupBox("LUỒNG CAMERA NHẬN DIỆN CỬ CHỈ")
        cam_box_layout = QVBoxLayout(self.camera_group)
        
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #0f172a; border-radius: 4px; border: 1px solid #1e293b;")
        cam_box_layout.addWidget(self.image_label)
        
        right_layout.addWidget(self.camera_group)
        main_layout.addLayout(right_layout, 2)

        # --- ĐĂNG KÝ ĐỒNG BỘ CHUỘT ---
        self.master_slider.valueChanged.connect(self.on_slider_changed)
        self.btn_up.clicked.connect(lambda: self.manual_publish(-self.manual_speed, 0.0))
        self.btn_down.clicked.connect(lambda: self.manual_publish(self.manual_speed, 0.0))
        self.btn_left.clicked.connect(lambda: self.manual_publish(0.0, -self.angular_speed))  
        self.btn_right.clicked.connect(lambda: self.manual_publish(0.0, self.angular_speed))   
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_reset.clicked.connect(self.emergency_stop)
        self.btn_toggle_ai.clicked.connect(self.toggle_mode)

        # --- KHÓA FOCUS CÁC NÚT ---
        self.master_slider.setFocusPolicy(Qt.NoFocus)
        self.btn_up.setFocusPolicy(Qt.NoFocus)
        self.btn_down.setFocusPolicy(Qt.NoFocus)
        self.btn_left.setFocusPolicy(Qt.NoFocus)
        self.btn_right.setFocusPolicy(Qt.NoFocus)
        self.btn_stop.setFocusPolicy(Qt.NoFocus)
        self.btn_reset.setFocusPolicy(Qt.NoFocus)
        self.btn_toggle_ai.setFocusPolicy(Qt.NoFocus)

        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    # =====================================================================
    # 3. HÀM XỬ LÝ SỰ KIỆN BÀN PHÍM WASD
    # =====================================================================
    def keyPressEvent(self, event):
        if self.current_mode != "MANUAL" or event.isAutoRepeat():
            return

        key = event.key()
        if key == Qt.Key_W:
            self.manual_publish(-self.manual_speed, 0.0)
        elif key == Qt.Key_S:
            self.manual_publish(self.manual_speed, 0.0)
        elif key == Qt.Key_A:
            self.manual_publish(0.0, -self.angular_speed)  
        elif key == Qt.Key_D:
            self.manual_publish(0.0, self.angular_speed)   
        elif key == Qt.Key_Space:
            self.emergency_stop()

    def keyReleaseEvent(self, event):
        if self.current_mode != "MANUAL" or event.isAutoRepeat():
            return
        
        if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            self.manual_publish(0.0, 0.0)

    # =====================================================================
    # 4. LOGIC ĐIỀU KHIỂN & PHÁT DỮ LIỆU ROS 2
    # =====================================================================
    def on_slider_changed(self, value):
        self.manual_speed = value / 100.0
        self.master_value_label.setText(f"{self.manual_speed:.2f} m/s")
        self.speed_display.display(self.manual_speed)

    def manual_publish(self, linear_v, angular_w):
        if self.current_mode == "MANUAL":
            msg = Twist()
            msg.linear.x = float(linear_v)
            msg.angular.z = float(angular_w)
            self.pub.publish(msg)

    def process_ai_velocity(self, v, w):
        if self.current_mode == "AI":
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub.publish(msg)

    def emergency_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def toggle_mode(self):
        if self.current_mode == "MANUAL":
            self.current_mode = "AI"
            self.current_mode_label.setText("CHẾ ĐỘ: TỰ HÀNH CỬ CHỈ TAY (AI VISION MODE)")
            self.current_mode_label.setStyleSheet("font-size: 13px; color: #00ffcc; font-weight: bold;")
            self.btn_toggle_ai.setText("CHUYỂN SANG: ĐIỀU KHIỂN THỦ CÔNG (MANUAL)")
            self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #f59e0b; color: white; padding: 12px; font-weight: bold; font-size: 13px; border-radius: 5px; }")
            self.btn_up.setDisabled(True)
            self.btn_down.setDisabled(True)
            self.btn_left.setDisabled(True)
            self.btn_right.setDisabled(True)
        else:
            self.emergency_stop() 
            self.current_mode = "MANUAL"
            self.current_mode_label.setText("CHẾ ĐỘ: ĐIỀU KHIỂN THỦ CÔNG (MANUAL - WASD)")
            self.current_mode_label.setStyleSheet("font-size: 13px; color: #eab308; font-weight: bold;")
            self.btn_toggle_ai.setText("KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION)")
            self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #4f46e5; color: white; padding: 12px; font-weight: bold; font-size: 13px; border-radius: 5px; }")
            self.btn_up.setEnabled(True)
            self.btn_down.setEnabled(True)
            self.btn_left.setEnabled(True)
            self.btn_right.setEnabled(True)
        
        self.setFocus()

    def update_camera_frame(self, qt_img):
        self.image_label.setPixmap(QPixmap.fromImage(qt_img).scaled(
            640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def closeEvent(self, event):
        self.emergency_stop()
        self.ai_thread.stop()
        rclpy.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ProDashboard()
    window.show()
    sys.exit(app.exec_())