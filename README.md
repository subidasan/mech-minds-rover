# MECH-MINDS: Vision-Based Autonomous 4WD Reconnaissance Rover
**Hack in Hills 26 (HiH 26) - Track 01: Vision-Based Autonomous Robot**

An edge-computing autonomous 4WD disaster reconnaissance rover and physics-synchronized Gazebo 3D Digital Twin designed to map and navigate hazardous environments without external cloud infrastructure.

## Project Deliverables Index
1. **Complete Source Code**:
   - edge_engine/ : 40Hz OpenCV/YOLOv8n perception pipeline, V-SLAM log-odds occupancy grid, 8-connected A* global planner, local potential fields, and C++17 hardware PID controller (motor_pid.cpp).
   - core_gateway/ : Node.js and Socket.io telemetry microservice.
   - mobile_dashboard/ : React Native (Expo) operator console (MECH-MINDS UI) featuring live HUD camera stream, interactive 2D occupancy map, custom Home Base station locking, and motor PWM telemetry charts.
   - simulation/ : Gazebo 3D Digital Twin (rover_world.sdf), custom Autodesk Fusion CAD mesh (rover_chassis.stl), and 33Hz kinematic bridge (gazebo_bridge.py).
2. **Hardware Component Specifications & Circuit Diagrams**:
   - [docs/HARDWARE_AND_CIRCUIT.md](docs/HARDWARE_AND_CIRCUIT.md)
3. **Simulation Documentation & Physical Deployment Plan**:
   - [docs/SIMULATION_AND_DEPLOYMENT.md](docs/SIMULATION_AND_DEPLOYMENT.md)
