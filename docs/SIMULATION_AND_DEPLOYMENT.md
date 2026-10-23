# Simulation Setup & Physical Hardware Implementation Plan

## 1. Gazebo 3D Digital Twin Architecture (simulation/)
- **Custom CAD Chassis (rover_chassis.stl)**: Designed in Autodesk Fusion and imported into rover_world.sdf with zero-sink kinematic ground and ramp elevation synchronization.
- **Hard-Mode Multi-Hazard Gauntlet**: Contains 16 dense obstacle zones and 3 elevated traversable ramps (ramp_alpha, ramp_bravo, ramp_charlie).
- **Real-Time Kinematic Bridge (gazebo_bridge.py)**: Reads velocity, pose, custom Home Base resets, and ramp pitch states from FastAPI (:8000/status) at 33Hz and updates Gazebo via /world/mech_minds_arena/set_pose.

## 2. Lifelong AI Hazard Memory & Multi-Disaster Autonomy (edge_engine/)
- **Persistent Spatial Memory (main_rover_service.py)**: Records close-range hazard encounters into an online memory buffer that survives mission resets, permanently penalizing high-risk coordinates in the global A* costmap.
- **Multi-Class Vision Segmentation (perception_gstreamer.py)**: Segments structural debris, bio-chemical spills, radiation zones, and traversable ramps from first-person camera frames using HSV contour masks and focal-length depth estimation.
- **Log-Odds Occupancy Grid & Anti-Stuck Guard (vslam_occupancy.py)**: Raycasts free corridors, applies class-specific safety inflation buffers around detected hazards, and guarantees a clear 3x3 rover footprint so A* never deadlocks in tight choke points.
- **Dynamic Anomaly Spawner & Cleanup**: Allows live injection and instant clearing of unmapped hazards ahead of the rover to validate reactive obstacle avoidance.

## 3. Simulation vs. Physical Hardware Transition

| Subsystem | Functional Simulation Mode (SIM_MODE=true) | Physical Hardware Mode (SIM_MODE=false) |
| :--- | :--- | :--- |
| **Perception Input** | 40Hz synthetic perspective renderer synced to rover pose | GStreamer v4l2src hardware camera pipeline on /dev/video0 |
| **Localization & Pitch** | Kinematic integration + raised-cosine ramp slope model | ORB visual odometry fused with MPU6050 I2C pitch/yaw measurements |
| **Motor Control** | C++17 PID controller streams computed PWM duty cycles to UI | C++17 PID controller writes directly to Linux /sys/class/gpio pins |
