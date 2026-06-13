import cv2 #thư viện OpenCV
import mediapipe as mp  #nhận diện tay
import time     #tính FPS


#class xử lý hand tracking
class handDetector():
    def __init__(self,mode = False, maxHands=2, detectionCon=0.5, trackCon = 0.5): 
        #mode: xử lý ảnh tĩnh hay video / maxHands: tối đa bao nhiêu bàn tay
        #detectionCon: độ tin cậy phát hiện tay / trackCon: độ tin cậy tracking
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon      
        #khởi tạo mediapipe
        self.mpHands = mp.solutions.hands    #gọi module nhận diện tay 
        self.hands = self.mpHands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.maxHands,
            model_complexity=1,
            min_detection_confidence=self.detectionCon,
            min_tracking_confidence=self.trackCon
            ) # modal nhận diện tay
        self.mpDraw = mp.solutions.drawing_utils         #vẽ các điểm landmark lên ảnh


    def findHands(self, img, draw=True):
        #chuyển màu, openCV dùng BGR, mediapipe dùng RGB ---> đổi
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        #print(results.multi_hand_landmarks)
            
        if self.results.multi_hand_landmarks:      #kiểm tra xem có tay không
            for handLms in self.results.multi_hand_landmarks:     #duyệt từng bàn tay
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, 
                                               self.mpHands.HAND_CONNECTIONS)    #vẽ các điểm và các đường nối
        return img
    
    def findPosition(self, img, handNo = 0, draw = True):       #lấy tọa độ các điểm tay
        lmList = []    #list lưu điểm landmark
        if self.results.multi_hand_landmarks:        #kiểm tra xem có tay không
            myHand = self.results.multi_hand_landmarks [handNo]      #lấy 1 bàn tay
            for id, lm in enumerate(myHand.landmark):       #duyệt 21 điểm, mỗi điểm có id (0->20) và lm.x, lm.y dưới dạng %
                #print(id, lm) 
                #chuyển sang pixel
                h, w, c = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                #print(id, cx, cy)
                lmList.append([id, cx, cy])     #lưu vào list
                if draw:
                    cv2.circle(img, (cx,cy), 5, (255,0,255), cv2.FILLED)     #vẽ điểm BGR
        return lmList
    
def main():
    pTime = 0       #previous Time
    cTime = 0       #current Time
    cap = cv2.VideoCapture(0)       #mở webcam
    detector = handDetector()      #gọi detector
    
    while True:
        success, img = cap.read()       #đọc ảnh
        img = detector.findHands(img)       #nhận diện tay
        lmList = detector.findPosition(img)         #lấy tọa độ
        if len(lmList) != 0:
            print(lmList[4])    #in điểm số 4(ngón cái)
        #tính FPS
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        cv2.putText(img, str(int(fps)),(10,70), cv2.FONT_HERSHEY_PLAIN, 3,(255,0,255),3)        #hiển thị FPS
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        
if __name__ == "__main__":
    main()