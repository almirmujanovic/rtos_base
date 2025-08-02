import cv2
cap = cv2.VideoCapture("rtsp://192.168.137.223:8554/mjpeg/1")

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('ESP32 RTSP', frame)
        print("✅ Frame received!")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("❌ No frame")

cap.release()
cv2.destroyAllWindows()