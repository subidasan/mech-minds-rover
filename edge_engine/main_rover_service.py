import os
import math
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
    "home_cell": [30, 6],
    "goal_cell": [30, 54],
    "goal_reached": False,
    "linear_v": 0.0,
    "angular_w": 0.0,
    "reset_flag": False,
    "reset_coords": [30.0, 6.0],
    "learned_hazard_zones": [],
    "learned_class_counts": {"DEBRIS": 0, "BIOCHEM": 0, "RADIATION": 0},
    "adaptation_score": 84,
    "mission_log": [
        {"time": time.strftime("%H:%M:%S"), "event": "NEURAL SLAM: Multi-Disaster Classifier Ready (Seismic, Bio-Chem, Radiation)"}
    ],
    "logged_keys": set(),
    "last_telemetry": {}
}


def add_mission_log(msg_key, message):
    if msg_key not in rover_state["logged_keys"]:
        rover_state["logged_keys"].add(msg_key)
        rover_state["mission_log"].insert(0, {
            "time": time.strftime("%H:%M:%S"),
            "event": message
        })
        rover_state["mission_log"] = rover_state["mission_log"][:14]


class GoalPayload(BaseModel):
    goal_x: float
    goal_y: float


class HomePayload(BaseModel):
    home_x: float
    home_y: float
    use_current_pose: bool = False


class ScenarioPayload(BaseModel):
    scenario: str = "EARTHQUAKE_COLLAPSE"
    spawn_anomaly_kind: str = ""


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
            pose_now, rover_state["goal_cell"], rover_state["home_cell"]
        )

        slam_data = slam_mapper.update_slam_and_map(
            frame, detections, perception.world_obstacles, z_elev, pitch
        )

        pose = slam_data["pose"]
        binary_grid = slam_data["binary_grid"]

        # Online Multi-Disaster Neural Memory Learning
        for det in detections:
            if det["type"] == "HAZARD" and det["distance_m"] < 2.2:
                dist_cells = det["distance_m"] / 0.20
                angle = pose["yaw"] + det["bearing_rad"]
                est_x = pose["x_float"] + dist_cells * math.cos(angle)
                est_y = pose["y_float"] + dist_cells * math.sin(angle)
                lx, ly = int(round(est_x)), int(round(est_y))
                for w_obs in perception.world_obstacles:
                    if math.hypot(est_x - w_obs["x"], est_y - w_obs["y"]) < 3.2:
                        lx, ly = int(round(w_obs["x"])), int(round(w_obs["y"]))
                        break
                lx, ly = max(2, min(57, lx)), max(2, min(57, ly))
                kind = det.get("kind", "DEBRIS")
                existing_coords = [[z["x"], z["y"]] for z in rover_state["learned_hazard_zones"]]
                if [lx, ly] not in existing_coords:
                    rover_state["learned_hazard_zones"].append({"x": lx, "y": ly, "kind": kind})
                    rover_state["learned_class_counts"][kind] = rover_state["learned_class_counts"].get(kind, 0) + 1
                    rover_state["adaptation_score"] = min(99, 84 + len(rover_state["learned_hazard_zones"]))
                    add_mission_log(
                        f"learn_{lx}_{ly}",
                        f"AI LEARNED [{kind}]: Mapped {det['label']} at ({lx},{ly}) -> Adapted safety perimeter"
                    )

        # Apply learned hazard radii to global A* occupancy grid
        for lz in rover_state["learned_hazard_zones"]:
            lx, ly, kind = lz["x"], lz["y"], lz["kind"]
            pad = 2 if kind in ("BIOCHEM", "RADIATION") else 1
            for dx in range(-pad, pad + 1):
                for dy in range(-pad, pad + 1):
                    if 0 <= lx + dx < 60 and 0 <= ly + dy < 60:
                        binary_grid[ly + dy, lx + dx] = 1

        global_path = navigator.a_star_search(
            binary_grid, (pose["x"], pose["y"]), tuple(rover_state["goal_cell"])
        )

        if rover_state["autonomous_enabled"]:
            v_cmd, w_cmd, lookahead, reached = navigator.potential_fields_control(
                pose, global_path, binary_grid, rover_state["goal_cell"]
            )
            rover_state["linear_v"] = v_cmd
            rover_state["angular_w"] = w_cmd
            if reached and not rover_state["goal_reached"]:
                add_mission_log(
                    f"goal_{rover_state['goal_cell'][0]}_{rover_state['goal_cell'][1]}_{int(now)//5}",
                    f"TARGET REACHED: Arrived at ({rover_state['goal_cell'][0]}, {rover_state['goal_cell'][1]})"
                )
            rover_state["goal_reached"] = reached
        else:
            v_cmd = rover_state["linear_v"]
            w_cmd = rover_state["angular_w"]
            lookahead = [pose["x"], pose["y"]]
            rover_state["goal_reached"] = False

        if not rover_state["autonomous_enabled"] and v_cmd == 0.0:
            drive_mode = "E-STOP ENGAGED"
            torque_boost = 1.0
        elif rover_state["goal_reached"]:
            drive_mode = "AT TARGET WAYPOINT"
            torque_boost = 1.0
        elif active_ramp:
            drive_mode = f"CLIMBING {active_ramp}"
            torque_boost = 1.45
        elif detections and detections[0].get("kind") in ("BIOCHEM", "RADIATION"):
            drive_mode = f"HAZMAT EVASION ({detections[0].get('kind')})"
            torque_boost = 1.25
        elif abs(w_cmd) > 0.35:
            drive_mode = "DYNAMIC OBSTACLE EVASION"
            torque_boost = 1.15
        else:
            drive_mode = "AUTONOMOUS RECON CRUISE"
            torque_boost = 1.0

        if pid_controller is not None:
            motor_out = pid_controller.apply_kinematics(
                v_cmd * torque_boost, w_cmd, 0.32, v_cmd * 0.85, v_cmd * 0.85, dt
            )
        else:
            motor_out = {"pwm_l": abs(v_cmd) * 200, "pwm_r": abs(v_cmd) * 200, "dir_l": "FWD", "dir_r": "FWD"}

        motor_out["pwm_l"] = round(min(255.0, float(motor_out.get("pwm_l", 0.0)) * 28.0), 1)
        motor_out["pwm_r"] = round(min(255.0, float(motor_out.get("pwm_r", 0.0)) * 28.0), 1)

        telemetry_packet = {
            "timestamp": round(now, 3),
            "scenario": perception.current_scenario,
            "pose": pose,
            "home_cell": rover_state["home_cell"],
            "active_ramp": active_ramp,
            "drive_mode": drive_mode,
            "safety_score": rover_state["adaptation_score"],
            "learned_zones": rover_state["learned_hazard_zones"],
            "learned_counts": rover_state["learned_class_counts"],
            "mission_log": rover_state["mission_log"],
            "vo_confidence": slam_data["vo_confidence"],
            "detections": detections,
            "a_star_path": [[int(p[0]), int(p[1])] for p in global_path],
            "scanned_obstacles": slam_data["scanned_obstacles"],
            "world_obstacles": perception.world_obstacles,
            "world_ramps": perception.world_ramps,
            "goal_cell": rover_state["goal_cell"],
            "goal_reached": rover_state["goal_reached"],
            "reset_flag": rover_state["reset_flag"],
            "reset_coords": rover_state["reset_coords"],
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

        await asyncio.sleep(0.03)


@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(autonomous_control_loop())
    yield
    task.cancel()


app = FastAPI(title="MECH-MINDS Multi-Disaster Edge AI API", lifespan=lifespan)
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
    gx = max(2, min(57, int(round(payload.goal_x))))
    gy = max(2, min(57, int(round(payload.goal_y))))
    rover_state["goal_cell"] = [gx, gy]
    rover_state["goal_reached"] = False
    rover_state["autonomous_enabled"] = True
    add_mission_log(
        f"goal_set_{gx}_{gy}_{int(time.time())}",
        f"ROUTE UPDATED: Navigating to ({gx}, {gy})"
    )
    return {"status": "GOAL_UPDATED", "goal_cell": rover_state["goal_cell"]}


@app.post("/home")
async def set_home_position(payload: HomePayload):
    if payload.use_current_pose:
        pose = slam_mapper.get_pose_state()
        hx, hy = pose["x"], pose["y"]
    else:
        hx = max(2, min(57, int(round(payload.home_x))))
        hy = max(2, min(57, int(round(payload.home_y))))
    rover_state["home_cell"] = [hx, hy]
    rover_state["reset_coords"] = [float(hx), float(hy)]
    add_mission_log(
        f"home_{hx}_{hy}_{int(time.time())}",
        f"HOME BASE LOCKED: Coordinates ({hx}, {hy}) saved as primary base station"
    )
    return {"status": "HOME_UPDATED", "home_cell": rover_state["home_cell"]}


@app.post("/scenario")
async def switch_disaster_scenario(payload: ScenarioPayload):
    if payload.spawn_anomaly_kind:
        pose = slam_mapper.get_pose_state()
        spawned = perception.spawn_dynamic_anomaly(
            pose["x_float"], pose["y_float"], pose["yaw"], payload.spawn_anomaly_kind
        )
        add_mission_log(
            f"spawn_{spawned['x']}_{spawned['y']}_{int(time.time())}",
            f"UNMAPPED ANOMALY: New {payload.spawn_anomaly_kind} hazard detected ahead at ({int(spawned['x'])},{int(spawned['y'])})!"
        )
        return {"status": "ANOMALY_SPAWNED", "obstacle": spawned}

    perception.load_scenario(payload.scenario)
    slam_mapper.log_odds.fill(0.0)
    rover_state["learned_hazard_zones"] = []
    add_mission_log(
        f"scen_{payload.scenario}_{int(time.time())}",
        f"ENVIRONMENT SWITCHED: Loaded {payload.scenario} hazard profile"
    )
    return {"status": "SCENARIO_LOADED", "scenario": payload.scenario}


@app.post("/override")
async def manual_override(payload: ManualOverridePayload):
    if payload.reset_sim:
        hx, hy = rover_state["home_cell"]
        slam_mapper.reset_map_and_pose(float(hx), float(hy), 1.5708, clear_map=False)
        rover_state["reset_coords"] = [float(hx), float(hy)]
        rover_state["reset_flag"] = True
        rover_state["goal_reached"] = False
        add_mission_log(
            f"reset_{int(time.time())}",
            f"DEPLOYED FROM HOME ({hx},{hy}): Retained {len(rover_state['learned_hazard_zones'])} learned hazard zones"
        )
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
