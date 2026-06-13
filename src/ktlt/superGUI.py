import sys
import cv2
import threading
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStackedWidget, QPushButton
from PyQt5.QtGui import QImage, QPixmap

# --- IMPORT MODULE XỬ LÝ TAY CỦA RIÊNG BẠN ---
import HandTrackingModule as htm

# --- IMPORT BẢN VẼ DESIGNER CỦA BẠN ---
from GUI import Ui_MainWindow

# =========================================================
# 1. LUỒNG XỬ LÝ CAMERA & AI (Sử dụng HandTrackingModule)
# =========================================================
class VideoThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QImage) 
    velocity_signal = QtCore.pyqtSignal(float, float) 

    def __init__(self):
        super().__init__()
        self._run_flag = True
        # Tái sử dụng chính xác cấu hình detector từ file cũ của bạn
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

            # Lật gương giống file HandSpeedPublisher.py
            img = cv2.flip(img, 1)
            
            # Nhận diện tay bằng module htm của bạn
            img = self.detector.findHands(img, draw=True)
            
            rpm_L = 0.0
            rpm_R = 0.0

            if self.detector.results.multi_hand_landmarks:
                num_hands = len(self.detector.results.multi_hand_landmarks)
                hands_data = []
                
                # Lấy tọa độ bằng hàm findPosition của bạn
                for i in range(num_hands):
                    lmList = self.detector.findPosition(img, handNo=i, draw=False)
                    if lmList:
                        hands_data.append(lmList)
                
                # Sắp xếp tay từ trái sang phải
                hands_data.sort(key=lambda hand: hand[0][1])

                for i, lmList in enumerate(hands_data):
                    x1, y1 = lmList[4][1], lmList[4][2]
                    x2, y2 = lmList[8][1], lmList[8][2]
                    length = math.hypot(x2 - x1, y2 - y1)
                    
                    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3) 
                    cv2.circle(img, (x1, y1), 8, (255, 0, 255), cv2.FILLED)
                    cv2.circle(img, (x2, y2), 8, (255, 0, 255), cv2.FILLED)
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)

                    rpm = np.interp(length, [30, 150], [0, 150]) 
                    
                    if lmList[20][2] > lmList[17][2]: 
                        rpm = -rpm 

                    if len(hands_data) == 2:
                        if i == 0: rpm_L = rpm
                        else: rpm_R = rpm
                    else:
                        rpm_L = rpm_R = rpm

                    text_x, text_y = lmList[0][1], lmList[0][2]
                    cv2.putText(img, f'{int(rpm)}', (text_x, text_y + 30), 
                                cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            # Động học thuận
            v_L = (rpm_L * 2 * math.pi / 60.0) * self.wheel_radius
            v_R = (rpm_R * 2 * math.pi / 60.0) * self.wheel_radius
            v = (v_R + v_L) / 2.0
            w = (v_R - v_L) / self.track_width

            cv2.putText(img, f'V: {v:.2f} m/s | W: {w:.2f} rad/s', (20, 40), 
                        cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)

            self.velocity_signal.emit(float(v), float(w))
            
            # Đẩy ảnh lên GUI (Đã bỏ hàm msleep gây lag)
            h, w_img, ch = img.shape
            img_drawn_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            convertToQtFormat = QImage(img_drawn_rgb.data, w_img, h, 3 * w_img, QImage.Format_RGB888)
            scaled_img = convertToQtFormat.scaled(700, 500, QtCore.Qt.KeepAspectRatio)
            self.change_pixmap_signal.emit(scaled_img)

        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# =========================================================
# 2. GIAO DIỆN TỔNG (QUẢN LÝ CHUYỂN TRANG)
# =========================================================
class SuperDashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard - HUST")
        self.resize(800, 700)
        self.setStyleSheet("background-color: #2b2b2b; color: white;")

        # --- Khởi tạo ROS 2 Node ---
        rclpy.init()
        self.node = Node('super_app_node')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True).start()

        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        self.page_selection = QWidget()
        self.page_ai = QWidget()
        self.page_manual = QtWidgets.QMainWindow()

        self.init_selection_page()
        self.init_ai_page()
        self.init_manual_page()

        self.stacked_widget.addWidget(self.page_selection) 
        self.stacked_widget.addWidget(self.page_ai)        
        self.stacked_widget.addWidget(self.page_manual)    

    def init_selection_page(self):
        layout = QVBoxLayout()
        label = QLabel("CHỌN PHƯƠNG THỨC ĐIỀU KHIỂN")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 26px; font-weight: bold; color: #007acc; margin-bottom: 30px;")
        
        btn_ai = QPushButton("1. ĐIỀU KHIỂN CỬ CHỈ TAY (AI)")
        btn_manual = QPushButton("2. ĐIỀU KHIỂN NÚT BẤM (MANUAL)")
        
        style = "padding: 40px; font-size: 20px; background-color: #444; border-radius: 15px; font-weight: bold;"
        btn_ai.setStyleSheet(style)
        btn_manual.setStyleSheet(style)

        btn_ai.clicked.connect(lambda: self.switch_page(1))
        btn_manual.clicked.connect(lambda: self.switch_page(2))

        layout.addStretch(); layout.addWidget(label); layout.addWidget(btn_ai); layout.addWidget(btn_manual); layout.addStretch()
        self.page_selection.setLayout(layout)

    def init_ai_page(self):
        layout = QVBoxLayout()
        self.lbl_cam = QLabel("Đang bật Camera...")
        self.lbl_cam.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_cam.setStyleSheet("border: 2px solid #007acc; background-color: black;")
        
        btn_back = QPushButton("QUAY LẠI MENU")
        btn_back.setStyleSheet("padding: 15px; background-color: #cc0000; border-radius: 10px; font-weight: bold;")
        btn_back.clicked.connect(lambda: self.switch_page(0))
        
        layout.addWidget(self.lbl_cam); layout.addWidget(btn_back)
        self.page_ai.setLayout(layout)

    def init_manual_page(self):
        self.manual_ui = Ui_MainWindow()
        self.manual_ui.setupUi(self.page_manual)

        self.manual_ui.pushButton.setStyleSheet("""
            QPushButton {
                background-color: #ff0000; 
                color: white; 
                border-radius: 45px; 
                font-weight: bold;
                font-size: 18px;
            }
            QPushButton:pressed { background-color: #aa0000; }
        """)

        self.manual_ui.velocity.setMinimum(5)
        self.manual_ui.velocity.setMaximum(55)
        self.manual_ui.velocity.setValue(20)
        self.manual_speed = 0.20
        self.manual_ui.label.setText(f"Tốc độ: {self.manual_speed:.2f} m/s")
        self.manual_ui.label.setStyleSheet("color: #00ff00; font-size: 16px; font-weight: bold;")
        self.manual_ui.label.setFixedWidth(200)
        
        pix = QPixmap("logo_hust.png")
        self.manual_ui.label_2.setPixmap(pix)

        self.btn_back_manual = QPushButton("QUAY LẠI MENU", self.page_manual)
        self.btn_back_manual.setGeometry(QtCore.QRect(20, 20, 150, 40)) 
        self.btn_back_manual.setStyleSheet("background-color: #555; border-radius: 5px; font-weight: bold;")
        self.btn_back_manual.clicked.connect(lambda: self.switch_page(0))

        self.manual_ui.velocity.valueChanged.connect(self.update_manual_speed)
        self.manual_ui.pushButton_3.clicked.connect(lambda: self.publish_cmd(self.manual_speed, 0.0))  
        self.manual_ui.pushButton_5.clicked.connect(lambda: self.publish_cmd(-self.manual_speed, 0.0)) 
        self.manual_ui.pushButton_2.clicked.connect(lambda: self.publish_cmd(0.0, 2.0))                
        self.manual_ui.pushButton_4.clicked.connect(lambda: self.publish_cmd(0.0, -2.0))               
        self.manual_ui.pushButton.clicked.connect(lambda: self.publish_cmd(0.0, 0.0))                 

    def update_manual_speed(self, val):
        self.manual_speed = val / 100.0
        self.manual_ui.label.setText(f"Tốc độ: {self.manual_speed:.2f} m/s")

    def update_ai_frame(self, q_img):
        self.lbl_cam.setPixmap(QPixmap.fromImage(q_img))

    def switch_page(self, index):
        self.stacked_widget.setCurrentIndex(index)
        if index == 1:
            self.ai_thread = VideoThread()
            self.ai_thread.change_pixmap_signal.connect(self.update_ai_frame)
            self.ai_thread.velocity_signal.connect(self.publish_cmd)
            self.ai_thread.start()
        elif hasattr(self, 'ai_thread'):
            self.ai_thread.stop() 
        
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dashboard = SuperDashboard()
    dashboard.show()
    sys.exit(app.exec_())