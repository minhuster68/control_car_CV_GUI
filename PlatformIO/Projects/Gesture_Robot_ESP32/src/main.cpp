#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

// --- Giữ nguyên CHÂN CẮM XE CỦA BẠN ---
#define ENCA_L 34  
#define ENCB_L 35  
#define ENA_L  14  
#define IN1_L  26  
#define IN2_L  27  

#define ENCA_R 18  
#define ENCB_R 19  
#define ENB_R  15  
#define IN3_R  4   
#define IN4_R  16  

#define LED_PIN 2

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmChannel_L = 0;
const int pwmChannel_R = 1;

const float PPR = 495;
long previousMillis = 0;

// Giữ nguyên thông số xe của bạn
const float L_TRACK = 0.20;   
const float R_WHEEL = 0.035;  

volatile long encoderCount_L = 0;
float targetRPM_L = 0.0, currentRPM_L = 0.0;
float eIntegral_L = 0, ePrevious_L = 0;
float Kp_L = 25.0, Ki_L = 15.0, Kd_L = 0.05; 

volatile long encoderCount_R = 0;
float targetRPM_R = 0.0, currentRPM_R = 0.0;
float eIntegral_R = 0, ePrevious_R = 0;
float Kp_R = 15.0, Ki_R = 10.0, Kd_R = 0.05;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = msg->linear.x;
  float w = msg->angular.z;

  float v_L = v - (w * L_TRACK / 2.0);
  float v_R = v + (w * L_TRACK / 2.0);

  targetRPM_L = (v_L / R_WHEEL) * (60.0 / (2.0 * PI));
  targetRPM_R = (v_R / R_WHEEL) * (60.0 / (2.0 * PI));
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
}

void IRAM_ATTR readEncoder_L() {
  if (digitalRead(ENCB_L) > 0) encoderCount_L++; else encoderCount_L--;
}
void IRAM_ATTR readEncoder_R() {
  if (digitalRead(ENCB_R) > 0) encoderCount_R++; else encoderCount_R--;
}

void setMotor_L(int dir, int pwmVal) {
  ledcWrite(pwmChannel_L, pwmVal);
  if (pwmVal == 0) { 
    digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, LOW);
  } else if (dir == 1) { 
    digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  } else if (dir == -1) { 
    digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH);
  }
}

void setMotor_R(int dir, int pwmVal) {
  ledcWrite(pwmChannel_R, pwmVal);
  if (pwmVal == 0) { 
    digitalWrite(IN3_R, LOW); digitalWrite(IN4_R, LOW);
  } else if (dir == 1) { 
    digitalWrite(IN3_R, HIGH); digitalWrite(IN4_R, LOW);
  } else if (dir == -1) { 
    digitalWrite(IN3_R, LOW); digitalWrite(IN4_R, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  char ssid[] = "iPhone Minh6251";
  char pass[] = "minh236251";
  char ip[] = "172.20.10.4";
  set_microros_wifi_transports(ssid, pass, ip, 8888);
  
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoder_L, RISING);
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  ledcSetup(pwmChannel_L, pwmFreq, pwmResolution); ledcAttachPin(ENA_L, pwmChannel_L);

  pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoder_R, RISING);
  pinMode(IN3_R, OUTPUT); pinMode(IN4_R, OUTPUT);
  ledcSetup(pwmChannel_R, pwmFreq, pwmResolution); ledcAttachPin(ENB_R, pwmChannel_R);

  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_tank_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  previousMillis = millis(); 
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  long currentTime = millis();
  
  if (currentTime - previousMillis >= 30) {
    // Cải tiến: Tính deltaT an toàn, tránh lỗi nhảy thời gian
    float deltaT = ((float)(currentTime - previousMillis)) / 1000.0; 
    previousMillis = currentTime;

    // Cải tiến: Khóa ngắt để đọc xung Encoder an toàn tuyệt đối
    noInterrupts();
    long count_L = encoderCount_L; encoderCount_L = 0;
    long count_R = encoderCount_R; encoderCount_R = 0;
    interrupts();
    
    currentRPM_L = ((float)count_L / PPR) / deltaT * 60.0;
    currentRPM_R = ((float)count_R / PPR) / deltaT * 60.0;

    // --- PID TRÁI ---
    float error_L = targetRPM_L - currentRPM_L;
    eIntegral_L += (error_L * deltaT);
    // Cải tiến: Xiết chặt Integral về 255 thay vì 500 để chống vọt ga
    if (eIntegral_L > 255) eIntegral_L = 255; else if (eIntegral_L < -255) eIntegral_L = -255;
    // Cải tiến: Thêm ePrevious_L = 0 để xe phanh mượt hơn
    if (targetRPM_L == 0) { eIntegral_L = 0; ePrevious_L = 0; } 
    
    float eDerivative_L = (error_L - ePrevious_L) / deltaT;
    float u_L = (Kp_L * error_L) + (Ki_L * eIntegral_L) + (Kd_L * eDerivative_L);
    ePrevious_L = error_L;

    float pwr_L = fabs(u_L);
    if (pwr_L > 255) pwr_L = 255;
    
    int dir_L = (u_L >= 0) ? 1 : -1; 
    if (targetRPM_L == 0) pwr_L = 0;
    setMotor_L(dir_L, (int)pwr_L);

    // --- PID PHẢI ---
    float error_R = targetRPM_R - currentRPM_R;
    eIntegral_R += (error_R * deltaT);
    if (eIntegral_R > 255) eIntegral_R = 255; else if (eIntegral_R < -255) eIntegral_R = -255;
    if (targetRPM_R == 0) { eIntegral_R = 0; ePrevious_R = 0; }
    
    float eDerivative_R = (error_R - ePrevious_R) / deltaT;
    float u_R = (Kp_R * error_R) + (Ki_R * eIntegral_R) + (Kd_R * eDerivative_R);
    ePrevious_R = error_R;

    float pwr_R = fabs(u_R);
    if (pwr_R > 255) pwr_R = 255;
    
    int dir_R = (u_R >= 0) ? 1 : -1;
    if (targetRPM_R == 0) pwr_R = 0;
    setMotor_R(dir_R, (int)pwr_R);
  }
}