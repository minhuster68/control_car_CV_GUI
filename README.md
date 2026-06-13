Markdown

# HUST Autonomous Control Station Pro

Hệ thống trạm điều khiển mặt đất (Ground Control Station) giao diện tối (Dark Theme), tích hợp xử lý thị giác máy tính AI nhận diện cử chỉ tay (MediaPipe) kết hợp bảng điều khiển thủ công (Manual Dashboard) qua mạng ROS 2 & Micro-ROS không dây để vận hành robot tự hành 3 bánh.

---

## 📌 1. Kiến Trúc Hệ Thống (System Architecture)

Hệ thống hoạt động dựa trên mô hình truyền nhận phân tầng thời gian thực:
* **Trạm điều khiển (PC - Ubuntu):** Chạy giao diện mã nguồn PyQt5, xử lý luồng AI xác định cử chỉ tay (`HandTrackingModule.py`) và đóng gói dữ liệu thành chuẩn `geometry_msgs/msg/Twist` bắn vào Topic `/cmd_vel`.
* **Cầu truyền thông (Cổng kết nối):** `micro_ros_agent` đóng vai trò trung gian mở cổng UDP tinh chỉnh mạng không dây.
* **Khối chấp hành (Xe Robot - ESP32):** Lắng nghe dữ liệu Topic `/cmd_vel`, giải mã động học nghịch thành tốc độ vòng/phút (RPM) mục tiêu cho từng bánh xe và chạy vòng quét PID siêu tốc (30ms) để điều tốc động cơ.

---

## 🔌 2. Sơ Đồ Đấu Nối Phần Cứng (Hardware Connection Setup)

Để đảm bảo hệ thống bám dòng xung ổn định, không bị nhiễu điện áp xung PWM hay sập nguồn đột ngột (Drop VIN) trên laptop và mạch, phần cứng phải được cấu hình đồng bộ Mass như sau:

### 2.1 Cấu Hình Khối Nguồn và Hạ Áp
* **Nguồn Pin chính (11.1V - 12V Li-Po/Acquy):**
  * Nối cực Dương (+) vào đầu **IN+** của mạch hạ áp LM2596.
  * Nối cực Dương (+) vào chân cấp nguồn động cơ (**VMOT** hoặc **12V**) của mạch Motor Driver.
* **Mạch hạ áp LM2596:**
  * Dùng tua vít căn chỉnh biến trở, dùng đồng hồ vạn năng đo ngõ ra đảm bảo đúng **5V**.
  * Nối đầu **OUT+ (5V)** vào chân **VIN** (hoặc 5V) của ESP32 để nuôi chip.
* **QUY TẮC BẮT BUỘC (GND Chung):** Tất cả các chân âm (-) hoặc chân GND của Pin, mạch LM2596, Motor Driver và ESP32 **phải được nối chụm lại với nhau**. Thiếu Mass chung sẽ làm mất xung PID và gây cháy hỏng vi điều khiển do dòng xả ngược từ động cơ.

### 2.2 Sơ Đồ Ghép Nối Chân Tín Hiệu (ESP32 Pinout)

| Thành phần ngoại vi | Chân ESP32 | Mô tả kỹ thuật |
| :--- | :---: | :--- |
| **LED_PIN** | `GPIO 2` | Đèn trạng thái tích hợp D2 (Báo kết nối/Heartbeat nhận xung) |
| **ENCA_L** (Encoder Bánh Trái) | `GPIO 34` | Đọc kênh xung A (Cấu hình Input Pullup, Ngắt RISING) |
| **ENCB_L** (Encoder Bánh Trái) | `GPIO 35` | Đọc kênh xung B để xác định chiều quay bánh trái |
| **ENA_L** (PWM Motor Trái) | `GPIO 14` | Cấp xung băm PWM điều tốc (Channel 0, Tần số 5kHz) |
| **IN1_L** (Direction Motor Trái)| `GPIO 26` | Chân logic điều khiển chiều quay bánh trái |
| **IN2_L** (Direction Motor Trái)| `GPIO 27` | Chân logic điều khiển chiều quay bánh trái |
| **ENCA_R** (Encoder Bánh Phải) | `GPIO 18` | Đọc kênh xung A (Cấu hình Input Pullup, Ngắt RISING) |
| **ENCB_R** (Encoder Bánh Phải) | `GPIO 19` | Đọc kênh xung B để xác định chiều quay bánh phải |
| **ENB_R** (PWM Motor Phải) | `GPIO 15` | Cấp xung băm PWM điều tốc (Channel 1, Tần số 5kHz) |
| **IN3_R** (Direction Motor Phải)| `GPIO 4`  | Chân logic điều khiển chiều quay bánh phải |
| **IN4_R** (Direction Motor Phải)| `GPIO 16` | Chân logic điều khiển chiều quay bánh phải |

---

## 💻 3. Thiết Lập Phần Mềm & Môi Trường (Software Installation)

### 3.1 Cấu Hình Trên Máy Tính (Ubuntu PC)
Yêu cầu hệ điều hành chạy ổn định môi trường ROS 2 (Humble/Foxy). Mở Terminal cài đặt các thư viện bổ trợ:

```bash
# Cập nhật hệ thống và cài đặt bộ công cụ dịch ngược UI
sudo apt update
sudo apt install python3-pip

# Cài đặt các thư viện Python chuyên dụng xử lý ảnh và AI
pip3 install pyqt5 opencv-python mediapipe numpy

3.2 Hướng Dẫn Kiểm Tra Địa Chỉ IP Trên Ubuntu

Để ESP32 có thể định tuyến chính xác gói tin UDP đến máy tính, bạn cần lấy địa chỉ IP của Laptop trong mạng Wi-Fi bằng cách mở Terminal và chạy lệnh sau:
Bash

hostname -I

Cách đọc kết quả:

    Lệnh sẽ trả về các dãy số IP phân cách bằng dấu cách. Bạn chỉ cần sao chép dãy số đầu tiên ngoài cùng bên trái (Ví dụ: 172.20.10.4). Đó chính là địa chỉ mạng hiện tại của Laptop.

3.3 Cấu Hình Mã Nguồn Trên Mạch ESP32 (Sử Dụng Wi-Fi Phát Từ Điện Thoại)

Để đảm bảo đường truyền không dây không bị chặn bởi tường lửa mạng công cộng hoặc cơ chế cô lập thiết bị, bắt buộc phải sử dụng Điểm phát sóng di động (Mobile Hotspot) từ điện thoại cá nhân để cả Laptop và ESP32 cùng kết nối vào.

Mở mã nguồn C++ (main.cpp) của ESP32, tìm đến khối cấu hình Wi-Fi trong hàm setup() và cập nhật thông số:
C++

char ssid[] = "TÊN_WIFI_ĐIỆN_THOẠI_CỦA_BẠN";
char pass[] = "MẬT_KHẨU_WIFI_ĐIỆN_THOẠI";
char ip[]   = "IP_MÁY_TÍNH_UBUNTU_VỪA_CHECK"; // Điền dãy số đầu tiên thu được từ lệnh 'hostname -I'

🚀 4. Hướng Dẫn Vận Hành Hệ Thống (Deployment Steps)

Thực hiện chuẩn xác theo chuỗi 3 bước sau để kích hoạt hệ thống:
Bước 1: Tắt tường lửa hệ thống và mở cổng kết nối Micro-ROS Agent

Mở một Terminal mới trên máy tính Ubuntu để mở cổng nhận dữ liệu không dây truyền từ xe:
Bash

sudo ufw disable
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

Bước 2: Khởi động xe Robot ESP32 và đồng bộ kết nối

    Cấp nguồn pin cho xe robot.

    Bắt buộc: Ngay sau khi lệnh khởi động Agent ở Bước 1 chuyển sang trạng thái running..., bạn phải bấm nút EN (Reset) trên bo mạch ESP32 để mạch thực hiện chu trình khởi động lại và gõ cửa trạm kết nối.

    Đèn LED D2 trên ESP32 sẽ nháy chậm (500ms/lần) để báo hiệu đang dò tìm mạng. Khi kết nối thông suốt, Terminal Agent sẽ nhảy thông báo session established. Lúc này đèn D2 sẽ tắt hẳn để chuyển sang chế độ chờ nhận lệnh điều tốc.

Bước 3: Cấu hình biến môi trường và chạy giao diện điều khiển

Mở một Terminal hoàn toàn mới tại thư mục chứa mã nguồn Python, chạy các lệnh cấu hình mạng truyền thông ngoài phân vùng Localhost cho ROS 2 và khởi động giao diện chính:
Bash

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
python3 pro_dashboard.py

🛠️ 5. Các Chế Độ Điều Khiển Trên Giao Diện (Control Modes)

Giao diện áp dụng cơ chế quản lý Quyền điều khiển tối cao (Control Authority) để bảo vệ an toàn cho robot:
1. Chế độ Thủ Công (MANUAL CONTROL MODE)

    Giao diện D-Pad: Sử dụng hệ thống 5 nút điều hướng chữ thập: TIẾN, LÙI, TRÁI, PHẢI và DỪNG.

    Thanh Ga Tổng (Speed Slider): Cho phép giới hạn dải tốc độ tối đa của xe dựa trên thông số cơ khí thực tế (Bán kính bánh xe R=0.035 m, Tốc độ động cơ tối đa 150 RPM→vmax​=0.55 m/s). Thanh trượt nhận giá trị nguyên từ 5 đến 55 tương ứng dải vận tốc an toàn từ 0.05 m/s đến 0.55 m/s.

2. Chế độ Cử Chỉ Tay (AI VISION GESTURE MODE)

    Nhấn nút màu tím KÍCH HOẠT: ĐIỀU KHIỂN CỬ CHỈ TAY (AI VISION).

    Cơ chế khóa an toàn (Control Locking): Giao diện sẽ lập tức làm mờ (Grey-out) và khóa toàn bộ chức năng của Slider và các nút bấm D-pad thủ công để tránh xung đột dữ liệu điều khiển.

    Nguyên lý bướm ga AI: * Khoảng cách pixel giữa đầu ngón cái (Landmark 4) và ngón trỏ (Landmark 8) được ánh xạ tuyến tính thành dải tốc độ động cơ tự động từ 0 đến 150 RPM.

        Nếu gập ngón út xuống (Tọa độ Y của Landmark 20 lớn hơn Landmark 17), thuật toán sẽ nhận diện xe vào số lùi (Đảo dấu RPM thành số âm).

    Màn hình hiển thị: Tốc độ thực tế RPM được tính toán độc lập cho từng bàn tay (Tay trái - Bánh trái, Tay phải - Bánh phải) và in trực tiếp lên chóp ngón trỏ. Vận tốc tổng hợp V,W được giám sát ở góc trên màn hình luồng Video.
