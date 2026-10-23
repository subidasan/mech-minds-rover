# Hardware Component Specifications & Circuit Diagrams (MECH-MINDS)

## 1. Bill of Materials (BOM)

| Component | Model / Part Number | Technical Specifications | Role in System |
| :--- | :--- | :--- | :--- |
| **Edge Compute Unit** | Raspberry Pi 4 Model B (8GB) | Quad-core Cortex-A72 @ 1.8GHz, 8GB LPDDR4 | Executes 40Hz perception, V-SLAM occupancy mapping, A* routing, and C++17 PID control |
| **Vision Sensor** | Arducam 8MP IMX219 Wide-Angle | 1080p30 / 720p60, 78 deg HFOV, CSI-2 Interface | Captures forward terrain to classify structural debris, chemical hazards, and climbable ramps |
| **Motor Driver** | L298N Dual Full-Bridge Driver | 5V-35V Drive Voltage, 2A Continuous per channel | Drives Left (M1+M2) and Right (M3+M4) 4WD DC motor banks |
| **Drive Actuators (4x)** | 12V 300RPM DC Geared Motors | 4.5 kg-cm stall torque, metal gearbox, 80mm tires | Differential skid-steering and high-torque ramp traversal |
| **Inertial Sensor** | MPU6050 6-DOF IMU | I2C interface (400kHz), 3-axis gyro + accelerometer | Measures chassis pitch angle on slopes and yaw heading |
| **Power System** | 3S 11.1V 5200mAh LiPo + XL4015 Buck | 11.1V motor rail; isolated 5.1V 5A logic rail | Prevents logic brownouts during peak motor stall currents |
| **Chassis Frame** | Custom Autodesk Fusion 3D Mesh | rover_chassis.stl (420mm x 280mm x 120mm, 4.5kg) | High-clearance 4WD mechanical chassis |

## 2. Raspberry Pi 4 BCM GPIO Pinout (edge_engine/motor_pid.cpp)

| BCM GPIO Pin | Physical Board Pin | Connected Component Pin | Signal Type & Function |
| :--- | :--- | :--- | :--- |
| **GPIO 17** | Pin 11 | L298N IN1 | Digital Output - Left Motor Bank Forward |
| **GPIO 27** | Pin 13 | L298N IN2 | Digital Output - Left Motor Bank Reverse |
| **GPIO 22** | Pin 15 | L298N IN3 | Digital Output - Right Motor Bank Forward |
| **GPIO 23** | Pin 16 | L298N IN4 | Digital Output - Right Motor Bank Reverse |
| **GPIO 12 (PWM0)** | Pin 32 | L298N ENA | Hardware PWM (0-255) - Left Bank Speed |
| **GPIO 13 (PWM1)** | Pin 33 | L298N ENB | Hardware PWM (0-255) - Right Bank Speed |
| **GPIO 2 / GPIO 3** | Pin 3 / Pin 5 | MPU6050 SDA / SCL | I2C1 Bus - Chassis Pitch & Yaw Telemetry |
| **GND** | Pins 6, 9, 14, 20 | Common Ground | Shared ground across Raspberry Pi, Buck Converter, and L298N |
