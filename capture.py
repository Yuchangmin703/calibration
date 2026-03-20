import cv2
import os

cap = cv2.VideoCapture(2, cv2.CAP_V4L2) # /dev/video2 <----- 비디오 번호 입력하면 됨.
                          # v4l2-ctl --list-devices 명령어로 연결된 카메라 확인 가능.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

save_dir = os.path.expanduser("~/calibration/intrinsic_images")
os.makedirs(save_dir, exist_ok=True)

count = 0
print("SPACE: 촬영 / Q: 종료")
while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 읽기 실패")
        break
    cv2.imshow("Calibration Capture", frame)
    key = cv2.waitKey(1)
    if key == ord(' '):
        path = os.path.join(save_dir, f"calib_{count:03d}.png")
        cv2.imwrite(path, frame)
        print(f"[{count}] Saved: {path}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Total: {count} images saved")

# python3 ~/calibration/capture.py <----- 카메라 키고 터미널에서 실행

