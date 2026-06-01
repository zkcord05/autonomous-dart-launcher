import cv2
import serial
import time
import numpy as np
import math

# ── SETTINGS ──────────────────────────────────────────
SERIAL_PORT  = 'COM4'
BAUD_RATE    = 115200
CAMERA_INDEX = 1

# HSV range for orange target
LOWER_COLOR = np.array([0,  120, 120])
UPPER_COLOR = np.array([300, 255, 255])

# Tracking
KP_MIN           = 0.003
KP_MAX           = 0.003
DEADBAND         = 20
CLOSE_THRESHOLD  = 30
MEDIUM_THRESHOLD = 80

# Trajectory constants
V0 = 34.5
G  = 9.81
H  = 1.016

# Camera calibration
FOCAL_LENGTH    = 800
TARGET_DIAMETER = 0.04

# Auto fire settings
AIM_THRESHOLD = 20
AIM_HOLD_TIME = 1.5
MIN_FIRE_DIST = 0.5
MAX_FIRE_DIST = 5.0
# ──────────────────────────────────────────────────────

def calculate_tilt_angle(distance, height=H, v0=V0):
    if distance <= 0:
        return 180
    best_angle = 0
    best_error = float('inf')
    for angle_deg in np.arange(-20, 45, 0.1):
        theta = math.radians(angle_deg)
        vx = v0 * math.cos(theta)
        vy = v0 * math.sin(theta)
        a = -0.5 * G
        b = vy
        c = height
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            continue
        t1 = (-b + math.sqrt(discriminant)) / (2*a)
        t2 = (-b - math.sqrt(discriminant)) / (2*a)
        t_flight = max(t1, t2)
        if t_flight <= 0:
            continue
        predicted_range = vx * t_flight
        error = abs(predicted_range - distance)
        if error < best_error:
            best_error = error
            best_angle = angle_deg

    # Flipped: add angle instead of subtract
    # so farther target = higher tilt (toward 90°)
    servo_angle = 180 - abs(best_angle)
    servo_angle = max(90, min(180, servo_angle))
    return servo_angle

def estimate_distance(apparent_diameter_pixels):
    if apparent_diameter_pixels <= 0:
        return None
    return (TARGET_DIAMETER * FOCAL_LENGTH) / apparent_diameter_pixels

def get_kp(error):
    dist = abs(error)
    if dist < DEADBAND:
        return 0
    elif dist < CLOSE_THRESHOLD:
        return KP_MIN
    elif dist < MEDIUM_THRESHOLD:
        t = (dist - CLOSE_THRESHOLD) / (MEDIUM_THRESHOLD - CLOSE_THRESHOLD)
        return KP_MIN + t * (KP_MAX - KP_MIN)
    else:
        return KP_MAX

class LauncherController:
    def __init__(self, port, baud):
        self.ser        = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print(f"Connected to ESP32 on {port}")
        self.pan_angle      = 90.0
        self.tilt_angle     = 180.0
        self.last_fire_time = 0
        self.fire_cooldown  = 3.0

    def send(self, cmd):
        self.ser.write(f"{cmd}\n".encode())

    def set_pan(self, angle):
        angle = max(0, min(180, angle))
        self.pan_angle = angle
        self.send(f"P{int(angle):03d}")

    def set_tilt(self, angle):
        angle = max(90, min(180, angle))
        self.tilt_angle = angle
        self.send(f"T{int(angle):03d}")

    def spin_up(self):
        self.send("X")

    def stop_motors(self):
        self.send("Z")

    def fire(self):
        now = time.time()
        if now - self.last_fire_time > self.fire_cooldown:
            self.send("F")
            self.last_fire_time = now
            return True
        return False

    def close(self):
        self.stop_motors()
        self.ser.close()

def main():
    launcher = LauncherController(SERIAL_PORT, BAUD_RATE)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    aim_start_time  = None
    fired_this_lock = False
    auto_fire       = False

    print("Tracker running!")
    print("Q=quit  F=manual fire  A=toggle auto-fire")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Rotate frame
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        h_frame, w_frame = frame.shape[:2]
        cx, cy = w_frame // 2, h_frame // 2

        # Orange detection
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_found    = False
        distance        = None
        tilt_correction = 180

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area > 500:
                target_found = True

                M  = cv2.moments(largest)
                tx = int(M["m10"] / M["m00"])
                ty = int(M["m01"] / M["m00"])

                # Distance estimation
                (ex, ey), (ea, eb), ang = cv2.fitEllipse(largest)
                apparent_diameter = max(ea, eb)
                distance = estimate_distance(apparent_diameter)

                error_x = tx - cx
                error_y = -(ty - cy)

                kp_x = get_kp(error_x)
                kp_y = get_kp(error_y)

                # Pan tracking
                if kp_x > 0:
                    new_pan = launcher.pan_angle - kp_x * error_x
                    launcher.set_pan(new_pan)

                # Tilt with trajectory correction (flipped)
                if distance and MIN_FIRE_DIST < distance < MAX_FIRE_DIST:
                    tilt_correction = calculate_tilt_angle(distance)
                    if kp_y > 0:
                        # flipped sign on error_y
                        new_tilt = tilt_correction + kp_y * error_y
                    else:
                        new_tilt = tilt_correction
                    launcher.set_tilt(new_tilt)
                else:
                    if kp_y > 0:
                        new_tilt = launcher.tilt_angle + kp_y * error_y
                        launcher.set_tilt(new_tilt)

                # Aim check
                aimed = (abs(error_x) < AIM_THRESHOLD and
                         abs(error_y) < AIM_THRESHOLD)

                if aimed and not fired_this_lock:
                    if aim_start_time is None:
                        aim_start_time = time.time()
                    hold_time = time.time() - aim_start_time
                    if auto_fire and hold_time >= AIM_HOLD_TIME:
                        if distance and MIN_FIRE_DIST < distance < MAX_FIRE_DIST:
                            fired = launcher.fire()
                            if fired:
                                fired_this_lock = True
                                launch_angle = 180 - tilt_correction
                                print(f"FIRED at {distance:.2f}m angle {launch_angle:.1f}deg")
                else:
                    aim_start_time  = None
                    fired_this_lock = False

                # Draw target info
                cv2.circle(frame, (tx, ty), 10, (0, 165, 255), 2)
                cv2.circle(frame, (tx, ty), 3,  (0, 165, 255), -1)
                cv2.line(frame,   (cx, cy), (tx, ty), (0, 255, 255), 1)

                if distance:
                    launch_ang = 180 - tilt_correction
                    cv2.putText(frame,
                        f"Dist:{distance:.2f}m Angle:{launch_ang:.1f}deg",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 165, 255), 2)

                cv2.putText(frame,
                    f"Error X:{error_x} Y:{error_y}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 165, 255), 2)

                cv2.putText(frame,
                    f"Pan:{int(launcher.pan_angle)} Tilt:{int(launcher.tilt_angle)}",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 165, 255), 2)

                if aimed:
                    if aim_start_time:
                        hold = time.time() - aim_start_time
                        cv2.putText(frame,
                            f"AIMED! {hold:.1f}s",
                            (10, 150), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 255), 2)

        # UI elements
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (255, 0, 0), 2)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (255, 0, 0), 2)
        cv2.circle(frame, (cx, cy), AIM_THRESHOLD, (0, 0, 255), 1)
        cv2.circle(frame, (cx, cy), DEADBAND,      (255, 255, 0), 1)

        # Mask preview
        mask_small = cv2.resize(mask, (160, 120))
        mask_bgr   = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        frame[0:120, 0:160] = mask_bgr

        # Status
        status = "TARGET FOUND" if target_found else "NO TARGET"
        color  = (0, 165, 255)  if target_found else (0, 0, 255)
        cv2.putText(frame, status,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, color, 2)

        # Auto fire indicator
        af_text  = "AUTO-FIRE: ON"  if auto_fire else "AUTO-FIRE: OFF"
        af_color = (0, 255, 0)      if auto_fire else (0, 0, 255)
        cv2.putText(frame, af_text,
                    (w_frame-200, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, af_color, 2)

        cv2.imshow("Orange Tracker + Launcher", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            launcher.fire()
            print("Manual fire!")
        elif key == ord('a'):
            auto_fire = not auto_fire
            print(f"Auto fire: {'ON' if auto_fire else 'OFF'}")

    cap.release()
    launcher.close()
    cv2.destroyAllWindows()
    print("Stopped.")

if __name__ == "__main__":
    main()
