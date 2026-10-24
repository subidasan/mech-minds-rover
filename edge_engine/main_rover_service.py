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
    "learned_class_counts": {"DEBRIS": 0, "BIOCHEM": 0, "RADIATION": 0, "FLOOD": 0},
    "adaptation_score": 84,
    "stuck_ticks": 0,
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
    clear_anomalies: bool = False
    cam_u: float = -1.0
    cam_v: float = -1.0
    target_x: float = -1.0
    target_y: float = -1.0


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

        # Apply learned hazard radii to global A* occupancy grid (keep compact pad=1 so corridors never seal shut)
        for lz in rover_state["learned_hazard_zones"]:
            lx, ly = lz["x"], lz["y"]
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if 0 <= lx + dx < 60 and 0 <= ly + dy < 60:
                        binary_grid[ly + dy, lx + dx] = 1

        # CRITICAL ANTI-STUCK GUARD: Always guarantee rover's immediate 3x3 cell footprint & goal are free space (0)
        # so A* never gets trapped inside an inflated hazard boundary!
        rx_i, ry_i = pose["x"], pose["y"]
        gx_i, gy_i = rover_state["goal_cell"][0], rover_state["goal_cell"][1]
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= rx_i + dx < 60 and 0 <= ry_i + dy < 60:
                    binary_grid[ry_i + dy, rx_i + dx] = 0
                if 0 <= gx_i + dx < 60 and 0 <= gy_i + dy < 60:
                    binary_grid[gy_i + dy, gx_i + dx] = 0

        global_path = navigator.a_star_search(
            binary_grid, (rx_i, ry_i), (gx_i, gy_i)
        )

        # Fallback direct line if all corridors are temporarily blocked
        if not global_path and not rover_state["goal_reached"]:
            global_path = [(rx_i, ry_i), (gx_i, gy_i)]

        if rover_state["autonomous_enabled"]:
            v_cmd, w_cmd, lookahead, reached = navigator.potential_fields_control(
                pose, global_path, binary_grid, rover_state["goal_cell"]
            )
            # Anti-Deadlock Recovery: If rover is not at goal but velocity drops near zero, apply forward escape impulse
            if not reached and abs(v_cmd) < 0.08:
                rover_state["stuck_ticks"] += 1
                if rover_state["stuck_ticks"] > 6:
                    v_cmd = 0.55
                    w_cmd = 0.65 if (int(now * 2) % 2 == 0) else -0.65
            else:
                rover_state["stuck_ticks"] = 0

            rover_state["linear_v"] = v_cmd
            rover_state["angular_w"] = w_cmd
            if reached and not rover_state["goal_reached"]:
                add_mission_log(
                    f"goal_{gx_i}_{gy_i}_{int(now)//5}",
                    f"TARGET REACHED: Arrived at ({gx_i}, {gy_i})"
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
        elif detections and detections[0].get("kind") in ("BIOCHEM", "RADIATION", "FLOOD"):
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
    if payload.clear_anomalies:
        perception.load_scenario(perception.current_scenario)
        slam_mapper.log_odds.fill(0.0)
        rover_state["learned_hazard_zones"] = []
        rover_state["learned_class_counts"] = {"DEBRIS": 0, "BIOCHEM": 0, "RADIATION": 0}
        rover_state["adaptation_score"] = 84
        add_mission_log(
            f"clear_{int(time.time())}",
            "ANOMALIES CLEARED: All spawned Bio-Chem/Rad hazards removed & occupancy grid refreshed"
        )
        return {"status": "ANOMALIES_CLEARED"}

    if payload.spawn_anomaly_kind:
        pose = slam_mapper.get_pose_state()
        cu = payload.cam_u if payload.cam_u >= 0.0 else None
        cv = payload.cam_v if payload.cam_v >= 0.0 else None
        tx = payload.target_x if payload.target_x >= 0.0 else None
        ty = payload.target_y if payload.target_y >= 0.0 else None
        spawned = perception.spawn_dynamic_anomaly(
            pose["x_float"], pose["y_float"], pose["yaw"], payload.spawn_anomaly_kind, cu, cv, tx, ty
        )
        add_mission_log(
            f"spawn_{spawned['x']}_{spawned['y']}_{int(time.time())}",
            f"UNMAPPED ANOMALY: New {payload.spawn_anomaly_kind} hazard detected ahead at ({int(spawned['x'])},{int(spawned['y'])})!"
        )
        return {"status": "ANOMALY_SPAWNED", "obstacle": spawned}

    perception.load_scenario(payload.scenario)
    slam_mapper.log_odds.fill(0.0)
    rover_state["learned_hazard_zones"] = []
    rover_state["learned_class_counts"] = {"DEBRIS": 0, "BIOCHEM": 0, "RADIATION": 0}
    add_mission_log(
        f"scen_{payload.scenario}_{int(time.time())}",
        f"ENVIRONMENT SWITCHED: Loaded {payload.scenario} hazard profile"
    )
    return {"status": "SCENARIO_LOADED", "scenario": payload.scenario}


@app.post("/override")
async def manual_override(payload: ManualOverridePayload):
    if payload.reset_sim:
        hx, hy = rover_state["home_cell"]
        slam_mapper.reset_map_and_pose(float(hx), float(hy), 1.5708, clear_map=True)
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


@app.get("/report")
async def export_mission_report():
    pose = slam_mapper.get_pose_state()
    zones = rover_state["learned_hazard_zones"]
    counts = rover_state["learned_class_counts"]
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    px, py = pose["x"], pose["y"]
    pz = pose.get("z_elevation", 0.0)
    hx, hy = rover_state["home_cell"][0], rover_state["home_cell"][1]
    gx, gy = rover_state["goal_cell"][0], rover_state["goal_cell"][1]
    score = rover_state["adaptation_score"]

    lines = [
        "# MECH-MINDS AUTONOMOUS RECONNAISSANCE - MISSION INCIDENT REPORT",
        f"**Timestamp:** {ts}",
        f"**Active Disaster Profile:** {perception.current_scenario}",
        f"**Rover Coordinates:** ({px}, {py}) | Elevation: {pz}m",
        f"**Home Base Station:** ({hx}, {hy})",
        f"**Target Sector:** ({gx}, {gy})",
        f"**AI Adaptation & Safety Rating:** {score}%",
        "",
        "## 1. Hazard Inventory Summary",
        f"- **Structural Debris (DEBRIS):** {counts.get('DEBRIS', 0)} zones",
        f"- **Bio-Chemical Plumes (BIOCHEM):** {counts.get('BIOCHEM', 0)} zones",
        f"- **Radiation Pockets (RADIATION):** {counts.get('RADIATION', 0)} zones",
        f"- **Submerged Flood Zones (FLOOD):** {counts.get('FLOOD', 0)} zones",
        "",
        "## 2. Geo-Tagged Hazard Coordinates (Lifelong Memory Log)",
    ]
    if not zones:
        lines.append("- No critical close-range hazards logged in current sortie.")
    else:
        for idx, z in enumerate(zones, 1):
            lines.append(f"{idx}. **[{z['kind']}]** Grid Sector ({z['x']}, {z['y']}) - Perimeter Inflated & Routed")
    lines.append("")
    lines.append("## 3. Black-Box Chronological Event Telemetry")
    for entry in rover_state["mission_log"]:
        lines.append(f"- `{entry['time']}` - {entry['event']}")
    report_md = "\n".join(lines)
    return {
        "status": "REPORT_GENERATED",
        "timestamp": ts,
        "scenario": perception.current_scenario,
        "safety_score": score,
        "counts": counts,
        "hazards": zones,
        "markdown": report_md
    }

@app.post("/memory/clear")
async def clear_spatial_memory():
    # 1. Wipe the V-SLAM log-odds occupancy matrix while keeping current rover pose
    slam_mapper.log_odds.fill(0.0)
    # 2. Clear learned hazard zones, class counters, and reset adaptation score to baseline
    rover_state["learned_hazard_zones"].clear()
    rover_state["learned_class_counts"] = {"DEBRIS": 0, "BIOCHEM": 0, "RADIATION": 0, "FLOOD": 0}
    rover_state["adaptation_score"] = 18.0
    # 3. Also clear any user-injected temporary anomalies so environment starts fresh
    if hasattr(perception, "clear_dynamic_hazards"):
        perception.clear_dynamic_hazards()
    log_event("SPATIAL MEMORY WIPED: Log-odds grid & hazard registry flushed for new environment")
    return {
        "status": "MEMORY_CLEARED",
        "adaptation_score": rover_state["adaptation_score"],
        "learned_zones": 0
    }

