import time
import shutil
import subprocess
import urllib.request
import json
import math

EDGE_STATUS_URL = "http://127.0.0.1:8000/status"
EDGE_POSE_URL = "http://127.0.0.1:8000/pose"
GZ_BIN = "gz" if shutil.which("gz") else ("ign" if shutil.which("ign") else None)

def body_pitch_and_yaw_to_quaternion(body_pitch, yaw):
    """Applies pitch strictly along the rover's forward nose axis (no sideways wheelies)."""
    # Quaternion for Yaw around Z: q_yaw = [0, 0, sin(yaw/2), cos(yaw/2)]
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # Quaternion for local Pitch around body Y-axis: q_pitch = [0, sin(p/2), 0, cos(p/2)]
    cp = math.cos(-body_pitch * 0.5)
    sp = math.sin(-body_pitch * 0.5)

    # Hamilton product q = q_yaw * q_pitch
    qw = cy * cp
    qx = -sy * sp
    qy = cy * sp
    qz = sy * cp
    return qx, qy, qz, qw

def run_bridge():
    print("[GazeboBridge] Zero-Sink Kinematic Lock Active (Smooth Ground & Ramp Climbing)...")
    grid_x = 30.0
    grid_y = 6.0
    yaw = 1.5708
    smooth_z = 0.085
    smooth_pitch = 0.0
    last_time = time.time()
    proc = None

    while True:
        try:
            now = time.time()
            dt = min(0.08, max(0.02, now - last_time))
            last_time = now

            with urllib.request.urlopen(EDGE_STATUS_URL, timeout=0.4) as response:
                telemetry = json.loads(response.read().decode("utf-8"))
                vel = telemetry.get("velocity", {})
                pose_info = telemetry.get("pose", {})
                lin_v = float(vel.get("linear", 0.0))
                ang_w = float(vel.get("angular", 0.0))
                target_z = float(pose_info.get("z", 0.085))
                target_pitch = float(pose_info.get("pitch", 0.0))
                if telemetry.get("reset_flag"):
                    grid_x, grid_y, yaw = 30.0, 6.0, 1.5708

            # Low-pass filter Z height and Pitch so ramp transitions are 100% butter-smooth
            smooth_z = 0.70 * smooth_z + 0.30 * target_z
            smooth_pitch = 0.70 * smooth_pitch + 0.30 * target_pitch

            yaw += ang_w * dt
            grid_x = max(2.0, min(57.0, grid_x + (lin_v * math.cos(yaw) * dt) / 0.20))
            grid_y = max(2.0, min(57.0, grid_y + (lin_v * math.sin(yaw) * dt) / 0.20))

            world_x = (grid_x - 30.0) * 0.20
            world_y = (grid_y - 6.0) * 0.20

            # Only spawn a new set_pose call if the previous one finished (prevents process queue jitter!)
            if GZ_BIN and (proc is None or proc.poll() is not None):
                qx, qy, qz, qw = body_pitch_and_yaw_to_quaternion(smooth_pitch, yaw)
                pose_req = (
                    f'name: "mech_minds_rover", '
                    f'position: {{x: {world_x:.3f}, y: {world_y:.3f}, z: {smooth_z:.3f}}}, '
                    f'orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}'
                )
                proc = subprocess.Popen(
                    [GZ_BIN, "service", "-s", "/world/mech_minds_arena/set_pose",
                     "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "60", "--req", pose_req],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

            pose_data = json.dumps({
                "x_float": round(grid_x, 2),
                "y_float": round(grid_y, 2),
                "yaw": round(yaw, 3)
            }).encode("utf-8")
            req = urllib.request.Request(EDGE_POSE_URL, data=pose_data, headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=0.2)

            print(
                f"[CAD Bridge] Pos:({grid_x:.1f},{grid_y:.1f}) Z:{smooth_z:.3f}m Pitch:{math.degrees(smooth_pitch):.1f}deg   ",
                end="\r"
            )
        except Exception as e:
            print(f"[CAD Bridge] Waiting for FastAPI (:8000)... ({e})   ", end="\r")

        time.sleep(0.035)

if __name__ == "__main__":
    run_bridge()
