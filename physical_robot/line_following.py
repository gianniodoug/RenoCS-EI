"""Pure computation from Basique_Navigation/suivi_ligne.py; original unchanged.

Same bottom 30%, threshold 80, three scan rows, nearest-centre groups, mean
centroid, PD history/gains and limits. Input is explicitly BGR uint8. BGR2GRAY
corrects the original RGB2GRAY/Picamera2 RGB888 channel-order mismatch; it is
equivalent to the original calculation with an actual RGB input. No devices.
"""
import cv2
import numpy as np


BASE_SPEED = 100
KP = 0.04
KD = 0.015
MAX_SPEED = 255


class LineFollower:
    def __init__(self):
        self.previous_error = 0

    def update(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        roi = gray[int(h * 0.7):h, :]
        _, binary = cv2.threshold(roi, 80, 255, cv2.THRESH_BINARY)
        scan_positions = [int(binary.shape[0] * 0.9),
                          int(binary.shape[0] * 0.75),
                          int(binary.shape[0] * 0.6)]
        centers = []
        for y in scan_positions:
            white_pixels = np.where(binary[y, :] > 0)[0]
            if len(white_pixels) > 0:
                groups = np.split(white_pixels, np.where(np.diff(white_pixels) > 1)[0] + 1)
                group_centers = [int((g[0] + g[-1]) / 2) for g in groups if len(g) > 0]
                cx_row = min(group_centers, key=lambda x: abs(x - w / 2))
                centers.append(cx_row)
        if not centers:
            return dict(found=False, left_pwm=0, right_pwm=0, center_px=None)
        cx = int(np.mean(centers))
        error = cx - w / 2
        derivative = error - self.previous_error
        correction = KP * error + KD * derivative
        self.previous_error = error
        left_speed = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
        return dict(found=True, left_pwm=left_speed, right_pwm=right_speed, center_px=cx)
