# autonomous-dart-launcher
Autonomous 2-DOF vision-guided dart launcher using OpenCV target tracking, ESP32 servo/motor control, and projectile kinematics trajectory correction.
# Autonomous Vision-Guided Dart Launcher

An autonomous 2-DOF turret system that detects, tracks, and engages a moving target using computer vision and projectile kinematics. Built from scratch as a sophomore mechanical engineering student.

## Demo Video
https://drive.google.com/drive/folders/1xppHd6eIwSyWc4KAM7h-KGw9dgBq9M6G?dmr=1&ec=wgc-drive-%5Bmodule%5D-goto

---

## System Overview

The system uses a camera mounted on a pan-tilt turret to detect an orange target, calculate the required trajectory correction based on estimated distance, and autonomously fire a foam dart to hit the target.

---

## Key Specifications

| Spec | Value |
|---|---|
| Muzzle velocity | 12.5 m/s average |
| Velocity consistency | ~93.6% |
| Tracking rate | 30 fps |
| Pan range | 180° |
| Tilt range | 90° (horizontal to vertical) |
| Fire cooldown | 3 seconds |
| Auto-fire hold time | 1.5 seconds |
| Working range | 0.5 - 5.0 meters |

---

## System Architecture
Camera (30fps)
↓
Python/OpenCV — HSV target detection
↓
Adaptive proportional controller
↓
Trajectory correction (projectile kinematics)
↓
Serial commands (UART 115200 baud)
↓
ESP32 firmware — real-time servo + motor control
↓
Pan servo + Tilt servo + Flywheel motors + Pusher servo

---

## Hardware

| Component | Spec | Purpose |
|---|---|---|
| ESP32 DevKit V1 | 240MHz dual core | Main controller |
| 2x MG996R servo | 10 kg·cm | Pan and tilt axes |
| SG90 servo | 1.8 kg·cm | Dart pusher |
| 2x 180-size DC motor | ~20,000 RPM | Flywheel launcher |
| L298N motor driver | 2A per channel | Motor speed control |
| LM2596 buck converter | 12V → 6V | Motor power supply |
| Logitech C270 webcam | 720p 30fps | Target detection |
| 5mW 650nm laser diode | Red dot | Aim indicator |

**3D Printed Components (PLA, Bambu printer):**
- Base plate with pan servo mount
- Pan yoke (U-bracket)
- Tilt platform
- Flywheel launcher housing
- Dart guide barrel
- Camera and laser mount

---

## Software Architecture

### Three-language system:

**Arduino C++ (ESP32 firmware)**
- Real-time servo control via PWM
- DC motor speed and direction control
- UART serial command parser
- Firing sequence with configurable spin-up time

**Python/OpenCV (host PC)**
- HSV color space target detection
- Adaptive proportional tracking controller
- Distance estimation from apparent target size
- Projectile trajectory correction calculation
- Serial communication with ESP32

**MATLAB (analysis)**
- Muzzle velocity characterization
- Trajectory modeling with and without drag
- Performance metrics and visualization
- PWM vs velocity calibration curve

---

## Trajectory Correction

The system calculates the required tilt angle to compensate for dart drop at distance:

```python
# For each candidate launch angle:
vx = v0 * cos(θ)
vy = v0 * sin(θ)

# Solve quadratic for time of flight:
t = (-vy + sqrt(vy² + 2*g*h)) / g

# Find angle that minimizes range error:
predicted_range = vx * t
```

Where:
- `v0 = 12.5 m/s` (measured muzzle velocity)
- `h = 1.016 m` (barrel height above floor)
- `g = 9.81 m/s²`

---

## Serial Command Protocol

Commands sent from Python to ESP32 over UART at 115200 baud:

| Command | Action |
|---|---|
| `P090` | Pan to 90° |
| `T175` | Tilt to 175° |
| `F` | Fire dart |
| `X` | Spin up flywheels |
| `Z` | Stop motors |
| `V200` | Set motor PWM to 200/255 |
| `U2000` | Set spin-up time to 2000ms |
| `C` | Center all servos |
| `?` | Request current angles |

---

## Wiring
Power:
12V 3A supply → LM2596 buck converter → 6V → L298N → DC motors
5V 1.5A supply → breadboard rails → servos + ESP32
ESP32 GPIO:
GPIO 13 → Pan servo signal
GPIO 16 → Tilt servo signal
GPIO 23 → Pusher servo signal
GPIO 25 → L298N ENA (motor 1 speed)
GPIO 26 → L298N IN1
GPIO 27 → L298N IN2
GPIO 32 → L298N ENB (motor 2 speed)
GPIO 33 → L298N IN3
GPIO 17 → L298N IN4

---

## Repository Structure
autonomous-dart-launcher/
├── arduino/
│   └── launcher.ino          # ESP32 firmware
├── python/
│   └── tracker.py            # Vision tracking + serial control
├── matlab/
│   └── trajectory_analysis.m # Muzzle velocity + trajectory modeling
├── media/
│   └── turret_overview.jpg   # Photos and screenshots
└── README.md

---

## Results

- Successfully tracked and engaged moving orange target autonomously
- Dart hit target at 1-3 meter range in autonomous fire mode
- Muzzle velocity characterized at 12.5 m/s with 93.6% shot-to-shot consistency
- Tracking maintained at up to 30fps with adaptive speed controller

---

## What I Would Improve

- Stereo camera or LiDAR for accurate depth estimation instead of apparent size
- Kalman filter for smoother target tracking under occlusion
- Brushless motors with encoder feedback for precise muzzle velocity control
- Closed-loop tilt angle verification using IMU
- Magazine-fed system for multiple autonomous shots

---

## Built With

- SolidWorks (mechanical design)
- Arduino IDE / C++ (ESP32 firmware)
- Python 3.13 + OpenCV 4.13 (computer vision)
- MATLAB (trajectory analysis)

---

## Author

**Zachary** — Sophomore Mechanical Engineering Student

*Personal project, May 2025*

