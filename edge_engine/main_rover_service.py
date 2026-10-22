import os
import asyncio
import time
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import socketio

from perception_gstreamer import EdgePerceptionEngine
from vslam_occupancy import VSLAMOccupancyMapper
from pathfinding_nav import AutonomousNavigator

try:
    import motor_pid
    HAS_CPP_PID = True
except ImportError:
    HAS_CPP_PID = False

SIM_MODE = os.getenv("SIM_MODE", "true").lower() == "true"
GATEWAY_URL = os.getenv("GATEWAY_URL", "http://localhost:4000")

sio = socketio.AsyncClient(reconnection=True)
perception = EdgePerceptionEngine(sim_mode=SIM_MODE)
slam_mapper = VSLAMOccupancyMapper(grid_size=60, resolution=0.20)
navigator = AutonomousNavigator(grid_size=60)

if HAS_CPP_PID:
    pid_controller = motor_pid.L298NPIDController(17, 27, 22, 23, 12, 13, 45.0, 2.5, 8.0, SIM_MODE)
else:
    pid_controller = None

rover_state = {
    "autonomous_enabled": True,
    "goal_cell": [30, 54],
    "goal_reached": False,
    "linear_v": 0.0,
    "angular_w": 0.0,
    "reset_flag": False,
    "last_telemetry": {}
}


class GoalPayload(BaseModel):
    goal_x: float
    goal_y: float


class ManualOverridePayload(BaseModel):
    autonomous_enabled: bool
    linear_v: float = 0.0
    angular_w: float = 0.0
    reset_sim: bool = False


class PosePayload(BaseModel):
    x_float: float
    y_float: float
    yaw: float


async def autonomous_control_loop():
    prev_time = time.time()
    while True:
        if not sio.connected:
            try:
                await sio.connect(GATEWAY_URL)
            except Exception:
                pass

        now = time.time()
        dt = max(0.015, min(0.10, now - prev_time))
        prev_time = now

        pose_now = slam_mapper.get_pose_state()
        frame, detections, frame_b64, z_elev, pitch, active_ramp = perception.process_frame(
            pose_now, rover_state["goal_cell"]
        )

        slam_data = slam_mapper.update_slam_and_map(
            frame, detections, perception.world_obstacles, z_elev, pitch
        )

        pose = slam_data["pose"]
        binary_grid = slam_data["binary_grid"]
        global_path = navigator.a_star_search(
            binary_grid, (pose["x"], pose["y"]), tuple(rover_state["goal_cell"])
        )

        if rover_state["autonomous_enabled"]:
            v_cmd, w_cmd, lookahead, reached = navigator.potential_fields_control(
                pose, global_path, binary_grid, rover_state["goal_cell"]
            )
            rover_state["linear_v"] = v_cmd
            rover_state["angular_w"] = w_cmd
            rover_state["goal_reached"] = reached
        else:
            v_cmd = rover_state["linear_v"]
            w_cmd = rover_state["angular_w"]
            lookahead = [pose["x"], pose["y"]]
            rover_state["goal_reached"] = False

        if pid_controller is not None:
            motor_out = pid_controller.apply_kinematics(v_cmd, w_cmd, 0.32, v_cmd * 0.90, v_cmd * 0.90, dt)
        else:
            motor_out = {"pwm_l": abs(v_cmd) * 200, "pwm_r": abs(v_cmd) * 200, "dir_l": "FWD", "dir_r": "FWD"}

        telemetry_packet = {
            "timestamp": round(now, 3),
            "pose": pose,
            "active_ramp": active_ramp,
            "vo_confidence": slam_data["vo_confidence"],
            "detections": detections,
            "a_star_path": [[int(p[0]), int(p[1])] for p in global_path],
            "scanned_obstacles": slam_data["scanned_obstacles"],
            "world_obstacles": perception.world_obstacles,
            "world_ramps": perception.world_ramps,
            "goal_cell": rover_state["goal_cell"],
            "goal_reached": rover_state["goal_reached"],
            "reset_flag": rover_state["reset_flag"],
            "lookahead_waypoint": lookahead,
            "velocity": {"linear": round(v_cmd, 3), "angular": round(w_cmd, 3)},
            "motors": motor_out,
            "camera_frame": frame_b64,
        }
        rover_state["reset_flag"] = False
        rover_state["last_telemetry"] = telemetry_packet

        if sio.connected:
            try:
                await sio.emit("edge_telemetry", telemetry_packet)
            except Exception:
                pass

        await asyncio.sleep(0.025)  # 40 Hz High-Speed Visual & Control Loop


@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(autonomous_control_loop())
    yield
    task.cancel()


app = FastAPI(title="MECH-MINDS Edge Perception & Navigation API", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/status")
async def get_status():
    return rover_state["last_telemetry"]


@app.post("/goal")
async def update_goal(payload: GoalPayload):
    rover_state["goal_cell"] = [
        max(2, min(57, int(round(payload.goal_x)))),
        max(2, min(57, int(round(payload.goal_y))))
    ]
    rover_state["goal_reached"] = False
    rover_state["autonomous_enabled"] = True
    return {"status": "GOAL_UPDATED", "goal_cell": rover_state["goal_cell"]}


@app.post("/override")
async def manual_override(payload: ManualOverridePayload):
    if payload.reset_sim:
        slam_mapper.reset_map_and_pose(30.0, 6.0, 1.5708)
        rover_state["reset_flag"] = True
        rover_state["goal_reached"] = False
    rover_state["autonomous_enabled"] = payload.autonomous_enabled
    rover_state["linear_v"] = payload.linear_v
    rover_state["angular_w"] = payload.angular_w
    if not payload.autonomous_enabled and payload.linear_v == 0.0 and pid_controller:
        pid_controller.emergency_stop()
    return {"status": "OVERRIDE_APPLIED", "state": rover_state["autonomous_enabled"]}


@app.post("/pose")
async def update_external_pose(payload: PosePayload):
    slam_mapper.rover_x_float = payload.x_float
    slam_mapper.rover_y_float = payload.y_float
    slam_mapper.rover_yaw = payload.yaw
    return {"status": "POSE_SYNCED"}
