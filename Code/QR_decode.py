import cv2
import subprocess
import time
from pyzbar.pyzbar import decode

def capture_qr_image():
    subprocess.run(["rpicam-jpeg", "-o", "QR.jpg"])

def scan_qr_code(image_path="QR.jpg"):
    while True:
        capture_qr_image()
        image = cv2.imread(image_path)
        if image is None:
            time.sleep(1)
            continue
        decoded_objects = decode(image)
        if not decoded_objects:
            time.sleep(1)
            continue
        for obj in decoded_objects:
            return obj.data.decode("utf-8")

if __name__ == "__main__":
    qr_text = scan_qr_code()
    print("QR Content:", qr_text)

