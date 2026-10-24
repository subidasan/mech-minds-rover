# MECH-MINDS: Vision-Based Autonomous 4WD Reconnaissance Rover
**Hack in Hills 26 (HiH 26) - Track 01: Vision-Based Autonomous Robot**

An edge-computing autonomous 4WD disaster reconnaissance rover and physics-synchronized Gazebo 3D Digital Twin designed to map, learn, and navigate hazardous, signal-denied environments without external cloud infrastructure.

---

## Core Technical Innovations
1. **Adaptive AI Hazard-Memory Mapping (Adaptive Experience Learning)**:
   - Unlike standard stateless planners that forget obstacles after a reset, the rover encodes newly encountered hazards into a persistent spatial memory layer.
   - On subsequent deployments from Home Base, the global A* planner automatically routes around previously learned trap zones and hazard perimeters.
2. **Multi-Disaster Vision Segmentation (Seismic, Bio-Chemical, Radiation, Submerged Floods & Overpass Ramps)**:
   - Real-time 40Hz OpenCV/YOLOv8n perception pipeline classifies four distinct terrain elements directly from camera frames: Structural Debris, Bio-Chemical Plumes, Radiation Zones, and Traversable Overpass Ramps.
   - Dynamically adjusts occupancy grid inflation radii based on threat severity (wider safety perimeters for bio-chemical and radiation pockets while keeping ramps open as elevated shortcuts).
3. **Real-Time Unmapped Anomaly Injection & Anti-Deadlock Recovery**:
   - Operators can spawn unmapped hazards directly ahead of the moving rover via the MECH-MINDS UI console to verify instantaneous sub-100ms trajectory replanning, footprint clearing, and one-click anomaly cleanup.
4. **Terrain-Adaptive C++17 PID & Ramp Torque Boost**:
   - Fuses raised-cosine slope pitch estimation with a pybind11 C++17 hardware PID controller, automatically applying a 1.45x PWM torque multiplier when climbing 3D disaster ramps.
5. **Automated Mission Incident Report Export & Customizable Home Base**:
   - Generates downloadable Markdown/JSON reconnaissance reports with geo-tagged hazard inventories, timestamps, and AI adaptation metrics.
   - Supports dynamic Home Base coordinate locking anywhere on the 2D V-SLAM grid, one-click Return-to-Home (RTH) routing, and an offline mission event recorder.

---

## Project Deliverables Index
1. **Complete Source Code**:
   - edge_engine/ : 40Hz OpenCV/YOLOv8n perception pipeline, V-SLAM log-odds occupancy grid, lifelong hazard memory, 8-connected A* global planner, local potential fields, and C++17 hardware PID controller (motor_pid.cpp).
   - core_gateway/ : Node.js and Socket.io telemetry microservice.
   - mobile_dashboard/ : React Native (Expo) operator console (MECH-MINDS UI) featuring live HUD camera stream, multi-disaster switcher, anomaly injector, interactive 2D occupancy map, custom Home Base station locking, and motor PWM telemetry charts.
   - simulation/ : Gazebo 3D Digital Twin (rover_world.sdf) with 16-obstacle gauntlet + 3 overpass ramps, custom Autodesk Fusion CAD mesh (rover_chassis.stl), and 33Hz kinematic bridge (gazebo_bridge.py).
2. **Hardware Component Specifications & Circuit Diagrams**:
   - [docs/HARDWARE_AND_CIRCUIT.md](docs/HARDWARE_AND_CIRCUIT.md)
3. **Simulation Documentation & Physical Deployment Plan**:
   - [docs/SIMULATION_AND_DEPLOYMENT.md](docs/SIMULATION_AND_DEPLOYMENT.md)

---

## Quick Start Commands
- **Terminal 1 (Edge AI Engine :8000)**: `cd edge_engine && source ~/.rover_venv_311/bin/activate && export SIM_MODE=true && uvicorn main_rover_service:app --host 0.0.0.0 --port 8000`
- **Terminal 2 (Node.js Gateway :4000)**: `cd core_gateway && node server.js`
- **Terminal 3 (MECH-MINDS UI :8081)**: `cd mobile_dashboard && node ./node_modules/expo/bin/cli start --web --clear`
- **Terminal 4 (Gazebo 3D Sim & Bridge)**: `cd simulation && export GZ_IP=127.0.0.1 && export LIBGL_ALWAYS_SOFTWARE=1 && gz sim -r rover_world.sdf & python3 gazebo_bridge.py`

---

##  Adaptive Spatial Memory & Black-Box Mission Auditing

To operate reliably in unpredictable, GPS-denied disaster zones, the MECH-MINDS Rover OS implements three critical field-readiness capabilities:

### 1. Blind-Spot Hazard Retention (Adaptive V-SLAM Memory)
* **Log-Odds Occupancy Locking:** As the rover traverses a sector, detected hazards (`DEBRIS`, `BIOCHEM`, `RADIATION`) within sensor range are geo-tagged and locked into a persistent 60x60 log-odds occupancy grid.
* **Out-of-Sight Rerouting:** Once a hazard is mapped, the rover retains its exact spatial perimeter even after driving past it (when the obstacle falls completely into the camera's blind spot). Subsequent sorties compute instant, collision-free A* trajectories using this retained spatial memory.

### 2. Dynamic Environment Memory Flush (` Wipe Memory` / `POST /memory/clear`)
* **Zero-Reboot Terrain Reset:** When redeploying the rover to a new disaster sector or after structural aftershocks reshape the arena, operators can flush stale spatial maps on the fly.
* **Instant Log-Odds Purge:** Clicking **` Wipe Memory`** zeroes the V-SLAM log-odds matrix, clears the hazard registry, and resets the AI Adaptation index without rebooting the edge service.

### 3. Automated Mission Incident Report (` Export Report` / `GET /report`)
* **One-Click Black-Box Audit:** Clicking **` Export Report`** downloads a formatted Markdown (`.md`) incident dossier containing the active disaster scenario, rover coordinates, AI adaptation score, geo-tagged hazard coordinates, and full chronological black-box telemetry log.
