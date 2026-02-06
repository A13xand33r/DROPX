import cv2
import subprocess
import numpy as np
import time

IMAGE_PATH = "landing.jpg"
ok = 0

def capture_image(retries=3, delay=1):
    for attempt in range(retries):
        try:
            subprocess.run(["rpicam-jpeg", "-o", IMAGE_PATH], check=True, timeout=15)
            return True
        except subprocess.TimeoutExpired:
            pass
        except subprocess.CalledProcessError:
            pass
        time.sleep(delay)
    return False

def detect_landing_pattern(template_path="template.jpg", threshold=0.6):
    if not capture_image():
        return False

    img_gray = cv2.imread(IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
    template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)

    if img_gray is None or template is None:
        return False

    scale_factor = 0.25
    img_small = cv2.resize(img_gray, (0, 0), fx=scale_factor, fy=scale_factor)
    t_h, t_w = template.shape
    found = None

    for s in np.linspace(0.5, 1.5, 10)[::-1]:
        resized = cv2.resize(img_small, (0, 0), fx=s, fy=s)
        if resized.shape[0] < t_h or resized.shape[1] < t_w:
            continue

        result = cv2.matchTemplate(resized, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(result)

        if max_val >= threshold:
            found = True
            break

    return bool(found)

if __name__ == "__main__":
    while ok == 0:
        if detect_landing_pattern():
            print("Landing pattern detected!")
            ok = 1
        else:
            print("Landing pattern NOT detected.")
        time.sleep(1)
