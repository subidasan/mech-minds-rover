# Simulation Setup & Physical Hardware Implementation Plan

## 1. Gazebo 3D Digital Twin Architecture (simulation/)
- **Custom CAD Chassis (rover_chassis.stl)**: Designed in Autodesk Fusion and imported into rover_world.sdf with zero-sink kinematic ground and ramp elevation synchronization.
- **Multi-Hazard Obstacle Gauntlet**: Contains 16 obstacle zones and 3 elevated traversable ramps (ramp_alpha, ramp_bravo, ramp_charlie).
- **Real-Time Kinematic Bridge (gazebo_bridge.py)**: Reads velocity, pose, and ramp pitch states from FastAPI (:8000/status) at 33Hz and updates Gazebo via /world/mech_minds_arena/set_pose.

## 2. Multi-Hazard Perception & Lifelong Mapping (edge_engine/)
- **Multi-Class Vision Segmentation (perception_gstreamer.py)**: Segments structural debris, bio-chemical spills, radiation zones, and traversable ramps from first-person camera frames using HSV contour masks and focal-length depth estimation.
- **Log-Odds Occupancy Grid (vslam_occupancy.py)**: Raycasts free corridors and applies class-specific safety inflation buffers around detected hazards while keeping ramps traversable for the 8-connected A* planner (pathfinding_nav.py).

## 3. Simulation vs. Physical Hardware Transition

| Subsystem | Functional Simulation Mode (SIM_MODE=true) | Physical Hardware Mode (SIM_MODE=false) |
| :--- | :--- | :--- |
| **Perception Input** | 40Hz synthetic perspective renderer synced to rover pose | GStreamer v4l2src hardware camera pipeline on /dev/video0 |
| **Localization & Pitch** | Kinematic integration + raised-cosine ramp slope model | ORB visual odometry fused with MPU6050 I2C pitch/yaw measurements |
| **Motor Control** | C++17 PID controller streams computed PWM duty cycles to UI | C++17 PID controller writes directly to Linux /sys/class/gpio pins |
