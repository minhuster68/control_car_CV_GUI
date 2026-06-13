import sys
import cv2
import threading
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QFormLayout, QGroupBox, QLabel, 
                             QPushButton, QSlider, QLCDNumber, QProgressBar, QGridLayout)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal

import HandTrackingModule as htm

# ============================================================================
# 1. LUỒNG XỬ LÝ CAMERA & AI DEEP LEARNING
# ============================================================================
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage) 
    velocity_signal = pyqtSignal(float, float) 

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.detector = htm.handDetector(maxHands=2, detectionCon=0.7)
        self.wheel_radius = 0.035
        self.track_width = 0.20

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)

        while self._run_flag:
            success, img = cap.read()
            if not success:
                continue

            img = cv2.flip(img, 1)
            img = self.detector.findHands(img, draw=True)
            
            rpm_L = 0.0
            rpm_R = 0.0

            if self.detector.results.multi_hand_landmarks:
                num_hands = len(self.detector.results.multi_hand_landmarks)
                hands_data = []
                
                for i in range(num_hands):
                    lmList = self.detector.findPosition(img, handNo=i, draw=False)
                    if lmList:
                        hands_data.append(lmList)
                
                hands_data.sort(key=lambda hand: hand[0][1])

                for i, lmList in enumerate(hands_data):
                    x1, y1 = lmList[4][1], lmList[4][2]
                    x2, y2 = lmList[8][1], lmList[8][2]
                    length = math.hypot(x2 - x1, y2 - y1)
                    
                    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3) 
                    cv2.circle(img, (x1, y1), 8, (255, 0, 255), cv2.FILLED)
                    cv2.circle(img, (x2, y2), 8, (255, 0, 255), cv2.FILLED)

                    rpm = np.interp(length, [30, 150], [0, 150]) 
                    if lmList[20][2] > lmList[17][2]: 
                        rpm = -rpm 

                    if len(hands_data) == 2:
                        if i == 0: rpm_L = rpm
                        else: rpm_R = rpm
                    else:
                        rpm_L = rpm_R = rpm

                    cv2.putText(img, f'RPM: {int(rpm)}', (lmList[8][1], lmList[8][2] - 20), 
                                cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            v_L = (rpm_L * 2 * math.pi / 60.0) * self.wheel_radius
            v_R = (rpm_R * 2 * math.pi / 60.0) * self.wheel_radius
            v = (v_R + v_L) / 2.0
            w = (v_R - v_L) / self.track_width

            self.velocity_signal.emit(float(v), float(w))
            
            h, w_img, ch = img.shape
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            qt_img = QImage(img_rgb.data, w_img, h, 3 * w_img, QImage.Format_RGB888)
            self.change_pixmap_signal.emit(qt_img)

        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# ============================================================================
# 2. GIAO DIỆN CHÍNH (MÔ PHỎNG THEO BẢN C++)
# ============================================================================
class ProDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_mode = "MANUAL" # Trạng thái: MANUAL hoặc AI
        self.manual_speed = 0.20
        
        self.init_ros()
        self.init_ui()
        
        # Bật Camera chạy nền liên tục
        self.ai_thread = VideoThread()
        self.ai_thread.change_pixmap_signal.connect(self.update_camera_frame)
        self.ai_thread.velocity_signal.connect(self.process_ai_velocity)
        self.ai_thread.start()

    def init_ros(self):
        rclpy.init()
        self.node = Node('pro_station_node')
        # Áp dụng QoS Best Effort cho Wi-Fi
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', qos_profile)
        threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True).start()

    def init_ui(self):
        self.setWindowTitle("HUST Autonomous Control Station Pro")
        self.resize(1200, 650)
        
        # Áp dụng chính xác Style Sheet từ file MainWindow.cpp
        self.setStyleSheet("""
            QMainWindow { background-color: #11121a; } 
            QLabel { color: #e2e8f0; font-family: 'Segoe UI'; font-size: 13px; } 
            QGroupBox { color: #00ffcc; font-weight: bold; font-size: 13px; border: 1px solid #334155; margin-top: 12px; border-radius: 6px; } 
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; } 
            QPushButton { background-color: #1e293b; color: #00ffcc; border: 1px solid #475569; border-radius: 4px; font-weight: bold; padding: 12px; } 
            QPushButton:pressed { background-color: #334155; }
        """)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # ==================================================
        # CỘT TRÁI (Tỉ lệ 4) - Bảng điều khiển & Giám sát
        # ==================================================
        left_layout = QVBoxLayout()
        left_layout.setSpacing(12)

        # 1. Khối Header Logo & Info
        header_layout = QHBoxLayout()
        logo_label = QLabel()
        pixmap = QPixmap("HUST.jpg") 
        if not pixmap.isNull():
            logo_label.setPixmap(pixmap.scaledToHeight(100, Qt.SmoothTransformation))
        
        title_label = QLabel()
        title_label.setStyleSheet("line-height: 1.4; font-family: 'Segoe UI';")
        title_label.setText("<span style='font-size: 15px; font-weight: bold; color: #f59e0b;'>ĐẠI HỌC BÁCH KHOA HÀ NỘI<br>"
                            "TRUNG TÂM ĐIỀU KHIỂN XE TỰ HÀNH</span><br>"
                            "<span style='font-size: 11px; font-weight: normal; color: #94a3b8;'>Sinh viên thực hiện: Minh</span><br>"
                            )
        
        header_layout.addWidget(logo_label)
        header_layout.addSpacing(10)
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        left_layout.addLayout(header_layout)

        # 2. Khối Giám sát (Telemetry Group)
        telemetry_group = QGroupBox("GIÁM SÁT THỜI GIAN THỰC")
        tele_layout = QFormLayout(telemetry_group)
        tele_layout.setContentsMargins(15, 15, 15, 15)
        tele_layout.setSpacing(10)
        
        self.speed_display = QLCDNumber()
        self.speed_display.setSegmentStyle(QLCDNumber.Flat)
        self.speed_display.setFixedSize(140, 45)
        self.speed_display.setStyleSheet("color: #00ffcc; background: #1e293b; border: 1px solid #475569; border-radius: 4px;")
        self.speed_display.display(self.manual_speed)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.battery_bar.setFixedSize(220, 22)
        self.battery_bar.setStyleSheet("QProgressBar { border: 1px solid #475569; border-radius: 4px; text-align: center; color: white; font-weight: bold; } "
                                       "QProgressBar::chunk { background-color: #10b981; border-radius: 3px; }")

        self.current_mode_label = QLabel("CHẾ ĐỘ: ĐIỀU KHIỂN THỦ CÔNG (MANUAL)")
        self.current_mode_label.setStyleSheet("font-size: 13px; color: #eab308; font-weight: bold;")

        tele_layout.addRow("VẬN TỐC HIỆN TẠI:", self.speed_display)
        tele_layout.addRow("MỨC NĂNG LƯỢNG:", self.battery_bar)
        tele_layout.addRow("TRẠNG THÁI HỆ THỐNG:", self.current_mode_label)
        left_layout.addWidget(telemetry_group)

        # 3. Khối Bảng Điều Khiển (Control Group)
        control_group = QGroupBox("BẢNG ĐIỀU KHIỂN HỆ THỐNG")
        control_group_layout = QVBoxLayout(control_group)
        control_group_layout.setContentsMargins(15, 15, 15, 15)
        control_group_layout.setSpacing(15)

        # Slider Ga Tổng
        master_layout = QHBoxLayout()
        master_title = QLabel("GA TỔNG (LIMIT):")
        master_title.setStyleSheet("font-weight: bold; color: #cbd5e1;")
        
        self.master_slider = QSlider(Qt.Horizontal)
        self.master_slider.setRange(5, 55)
        self.master_slider.setValue(20)
        self.master_slider.setStyleSheet("QSlider::groove:horizontal { background: #334155; height: 6px; border-radius: 3px; } "
                                         "QSlider::handle:horizontal { background: #f59e0b; width: 16px; height: 16px; margin: -5px 0; border-radius: 8px; }")
        
        self.master_value_label = QLabel(f"{self.manual_speed:.2f} m/s")
        self.master_value_label.setFixedWidth(60)
        self.master_value_label.setStyleSheet("font-weight: bold; color: #f59e0b; font-size: 14px;")

        master_layout.addWidget(master_title)
        master_layout.addWidget(self.master_slider)
        master_layout.addWidget(self.master_value_label)
        control_group_layout.addLayout(master_layout)

        # D-Pad Điều hướng
        btn_up = QPushButton("TIẾN (W)")
        btn_down = QPushButton("LÙI (S)")
        btn_left = QPushButton("TRÁI (A)")
        btn_right = QPushButton("PHẢI (D)")
        btn_stop = QPushButton("STOP (SPACE)")
        btn_stop.setStyleSheet("QPushButton { background-color: #b91c1c; color: white; border: none; } QPushButton:pressed { background-color: #7f1d1d; }")

        dpad_layout = QGridLayout()
        dpad_layout.setSpacing(8)
        dpad_layout.addWidget(btn_up,    0, 1)
        dpad_layout.addWidget(btn_left,  1, 0)
        dpad_layout.addWidget(btn_stop,  1, 1) 
        dpad_layout.addWidget(btn_right, 1, 2)
        dpad_layout.addWidget(btn_down,  2, 1)
        control_group_layout.addLayout(dpad_layout)
        left_layout.addWidget(control_group)

        # 4. Nút Chuyển Mode Khẩn Cấp (Thay thế cụm 4 Motor)
        self.btn_toggle_ai = QPushButton("KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION)")
        self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #4f46e5; color: white; padding: 15px; font-weight: bold; font-size: 14px; border-radius: 5px; } "
                                         "QPushButton:pressed { background-color: #3730a3; }")
        left_layout.addWidget(self.btn_toggle_ai)

        self.btn_reset = QPushButton("RESET HỆ THỐNG (EMERGENCY STOP)")
        self.btn_reset.setStyleSheet("QPushButton { background-color: #dc2626; color: white; padding: 12px; font-weight: bold; font-size: 14px; border-radius: 5px; border: none; } "
                                     "QPushButton:pressed { background-color: #991b1b; }")
        left_layout.addWidget(self.btn_reset)

        main_layout.addLayout(left_layout, 4)
        
        # ==================================================
        # CỘT PHẢI (Tỉ lệ 6) - Màn hình AI
        # ==================================================
        right_layout = QVBoxLayout()
        right_layout.setSpacing(8)
        
        cam_title = QLabel(" LUỒNG VIDEO CAMERA STREAM (MEDIAPIPE DEEP LEARNING) ")
        cam_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #00ffcc;")
        right_layout.addWidget(cam_title)

        self.camera_display = QLabel("ĐANG KẾT NỐI CAMERA...")
        self.camera_display.setAlignment(Qt.AlignCenter)
        self.camera_display.setStyleSheet("background-color: #090a0f; border: 2px dashed #00ffcc; border-radius: 8px;")
        right_layout.addWidget(self.camera_display, 1) # Cho phép label chiếm giãn toàn bộ khu vực

        main_layout.addLayout(right_layout, 6) 

        # --- KẾT NỐI SỰ KIỆN ---
        self.master_slider.valueChanged.connect(self.on_slider_changed)
        
        btn_up.clicked.connect(lambda: self.manual_publish(self.manual_speed, 0.0, "TIẾN ⬆️"))
        btn_down.clicked.connect(lambda: self.manual_publish(-self.manual_speed, 0.0, "LÙI ⬇️"))
        btn_left.clicked.connect(lambda: self.manual_publish(0.0, 2.5, "XOAY TRÁI ↩️"))
        btn_right.clicked.connect(lambda: self.manual_publish(0.0, -2.5, "XOAY PHẢI ↪️"))
        btn_stop.clicked.connect(self.emergency_stop)
        self.btn_reset.clicked.connect(self.emergency_stop)
        
        self.btn_toggle_ai.clicked.connect(self.toggle_mode)

    # ==================================================
    # CÁC HÀM XỬ LÝ LOGIC ĐIỀU KHIỂN
    # ==================================================
    def on_slider_changed(self, value):
        self.manual_speed = value / 100.0
        self.master_value_label.setText(f"{self.manual_speed:.2f} m/s")
        if self.current_mode == "MANUAL":
            self.speed_display.display(self.manual_speed)

    def toggle_mode(self):
        if self.current_mode == "MANUAL":
            self.current_mode = "AI"
            self.current_mode_label.setText("CHẾ ĐỘ: AI VISION ACTIVE 🧠")
            self.current_mode_label.setStyleSheet("color: #a855f7; font-weight: bold;")
            self.btn_toggle_ai.setText("TẮT AI - QUAY VỀ ĐIỀU KHIỂN THỦ CÔNG (MANUAL)")
            self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #64748b; color: white; padding: 15px; font-weight: bold; border-radius: 5px; }")
        else:
            self.current_mode = "MANUAL"
            self.current_mode_label.setText("CHẾ ĐỘ: ĐIỀU KHIỂN THỦ CÔNG (MANUAL)")
            self.current_mode_label.setStyleSheet("color: #eab308; font-weight: bold;")
            self.btn_toggle_ai.setText("KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION)")
            self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #4f46e5; color: white; padding: 15px; font-weight: bold; border-radius: 5px; }")
        
        self.publish_cmd(0.0, 0.0) # Dừng xe khi chuyển chế độ an toàn

    def emergency_stop(self):
        self.current_mode = "MANUAL"
        self.current_mode_label.setText("TRẠNG THÁI: ĐÃ DỪNG KHẨN CẤP 🛑")
        self.current_mode_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        self.speed_display.display(0)
        self.publish_cmd(0.0, 0.0)
        self.btn_toggle_ai.setText("KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION)")
        self.btn_toggle_ai.setStyleSheet("QPushButton { background-color: #4f46e5; color: white; padding: 15px; font-weight: bold; border-radius: 5px; }")

    def manual_publish(self, v, w, direction_text):
        if self.current_mode != "MANUAL":
            return # Block manual clicks if AI is active
        self.current_mode_label.setText(f"TRẠNG THÁI: XE ĐANG {direction_text}")
        self.current_mode_label.setStyleSheet("color: #10b981; font-weight: bold;")
        self.publish_cmd(v, w)

    def process_ai_velocity(self, v, w):
        if self.current_mode == "AI":
            self.speed_display.display(abs(v))
            self.publish_cmd(v, w)

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    # ==================================================
    # XỬ LÝ ĐỒ HỌA MÀN HÌNH AI
    # ==================================================
    def update_camera_frame(self, qt_img):
        # Scale ảnh theo kích thước của Label
        scaled_pixmap = QPixmap.fromImage(qt_img).scaled(
            800, 600,
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        self.camera_display.setPixmap(scaled_pixmap)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ProDashboard()
    window.show()
    sys.exit(app.exec_())