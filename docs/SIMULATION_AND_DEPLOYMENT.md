# Simulation Setup & Physical Hardware Implementation Plan
- **Gazebo 3D Digital Twin**: Synced via `simulation/gazebo_bridge.py` at 33Hz.
- **Perception**: 40Hz OpenCV/YOLOv8n contour & bounding-box depth estimation.
- **Navigation**: 8-connected A* Global Planner + Artificial Potential Fields + Smooth Ramp Elevation.
