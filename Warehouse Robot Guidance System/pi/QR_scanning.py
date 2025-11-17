import cv2
from pyzbar.pyzbar import decode
import time

def scan_qr_code():
    print("Attempting to open camera at /dev/video0...")
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)  # Use Video4Linux2 backend

    # Set resolution to 640x480 (or lower resolution if needed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open video device /dev/video0. Exiting...")
        return

    print("Camera opened successfully. Warming up...")
    time.sleep(2)

    print("Scanning for QR codes. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame. Exiting...")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = decode(gray)

        for qr_code in qr_codes:
            (x, y, w, h) = qr_code.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            qr_data = qr_code.data.decode("utf-8")
            print(f"QR Code detected: {qr_data}")
            cap.release()
            cv2.destroyAllWindows()
            return

        cv2.imshow('QR Code Scanner', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    scan_qr_code()
