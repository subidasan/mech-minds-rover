import os
import cv2
import math
import base64
import numpy as np


class EdgePerceptionEngine:
    def __init__(self, model_path="yolov8n_int8.tflite", sim_mode=False, width=480, height=280):
        self.width = width
        self.height = height
        self.sim_mode = sim_mode
        self.focal_length = 360.0
        self.known_obstacle_height = 0.50
        self.fov = math.radians(78.0)

        # 8 Disaster-Zone Debris Obstacles (Grid X, Grid Y) -> Untraversable (Red)
        self.world_obstacles = [
            (30.0, 18.0),
            (23.0, 24.0),
            (37.0, 25.0),
            (28.0, 32.0),
            (35.0, 36.0),
            (21.0, 39.0),
            (31.0, 44.0),
            (25.0, 49.0),
        ]

        # 2 Disaster Rubble Ramps (Grid X, Grid Y, Width Cells, Length Cells, Max Height m) -> Traversable!
        self.world_ramps = [
            {"x": 34.0, "y": 20.0, "w": 4.0, "l": 6.0, "h": 0.22, "label": "RAMP_ALPHA"},
            {"x": 26.0, "y": 41.0, "w": 4.0, "l": 6.0, "h": 0.22, "label": "RAMP_BRAVO"},
        ]

    def get_terrain_elevation_and_pitch(self, rx, ry):
        """Smooth cosine ramp profile: zero wheelies, zero sudden jumps."""
        base_z = 0.085  # Exact height where CAD wheels sit flush on ground
        for r in self.world_ramps:
            half_w = r["w"] / 2.0 + 0.4
            half_l = r["l"] / 2.0 + 0.4
            if abs(rx - r["x"]) <= half_w and abs(ry - r["y"]) <= half_l:
                # norm_y goes from -1.0 (bottom entry) -> 0.0 (top crest) -> +1.0 (bottom exit)
                norm_y = np.clip((ry - r["y"]) / half_l, -1.0, 1.0)
                # Smooth raised cosine elevation: 0 at edges, r["h"] at center
                z_elev = 0.5 * r["h"] * (1.0 + math.cos(math.pi * norm_y))
                # Smooth derivative for natural nose pitch (in radians)
                pitch = -0.16 * math.sin(math.pi * norm_y)
                return round(base_z + z_elev, 3), round(pitch, 3), r["label"]
        return base_z, 0.0, None

    def _render_first_person_camera(self, rx, ry, ryaw, pitch_offset, goal_cell):
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        horizon = int(self.height // 2 - 10 + pitch_offset * 120)
        horizon = max(30, min(self.height - 30, horizon))

        # Disaster sky & ground plane
        frame[:horizon, :] = (24, 20, 16)
        frame[horizon:, :] = (38, 44, 48)

        # Perspective ground grid lines
        for d in range(1, 14):
            proj_y = int(horizon + (self.focal_length * 0.20) / (d * 0.55))
            if horizon < proj_y < self.height:
                cv2.line(frame, (0, proj_y), (self.width, proj_y), (52, 60, 66), 1)

        # Collect all 3D objects (Obstacles + Ramps + Goal Beacon) for Painter's Algorithm
        render_queue = []

        if goal_cell:
            gdx = (goal_cell[0] - rx) * 0.20
            gdy = (goal_cell[1] - ry) * 0.20
            g_dist = math.hypot(gdx, gdy)
            g_rel = math.atan2(math.sin(math.atan2(gdy, gdx) - ryaw), math.cos(math.atan2(gdy, gdx) - ryaw))
            if abs(g_rel) < (self.fov / 1.8) and g_dist > 0.2:
                render_queue.append((g_dist, g_rel, "GOAL", None))

        for r in self.world_ramps:
            dx = (r["x"] - rx) * 0.20
            dy = (r["y"] - ry) * 0.20
            dist = math.hypot(dx, dy)
            rel_angle = math.atan2(math.sin(math.atan2(dy, dx) - ryaw), math.cos(math.atan2(dy, dx) - ryaw))
            if abs(rel_angle) < (self.fov / 1.7) and 0.30 < dist < 7.5:
                render_queue.append((dist, rel_angle, "RAMP", r))

        for ox, oy in self.world_obstacles:
            dx = (ox - rx) * 0.20
            dy = (oy - ry) * 0.20
            dist = math.hypot(dx, dy)
            rel_angle = math.atan2(math.sin(math.atan2(dy, dx) - ryaw), math.cos(math.atan2(dy, dx) - ryaw))
            if abs(rel_angle) < (self.fov / 1.7) and 0.25 < dist < 7.5:
                render_queue.append((dist, rel_angle, "HAZARD", (ox, oy)))

        render_queue.sort(key=lambda item: item[0], reverse=True)

        for dist, rel_angle, obj_type, meta in render_queue:
            cx = int((self.width / 2.0) - (rel_angle / (self.fov / 2.0)) * (self.width / 2.0))
            if obj_type == "GOAL":
                g_h = int(np.clip((0.5 * self.focal_length) / dist, 10, 140))
                cv2.circle(frame, (cx, horizon), max(6, g_h // 3), (128, 255, 0), -1)
            elif obj_type == "RAMP":
                rw = int(np.clip((0.80 * self.focal_length) / dist, 24, 260))
                rh = int(np.clip((0.22 * self.focal_length) / dist, 12, 110))
                cy = int(horizon + (self.focal_length * 0.18) / dist)
                pts = np.array([
                    [cx - rw // 2, cy + rh // 2],
                    [cx + rw // 2, cy + rh // 2],
                    [cx + rw // 3, cy - rh // 2],
                    [cx - rw // 3, cy - rh // 2],
                ], np.int32)
                # Draw traversable ramp in distinct Amber/Orange HSV color
                cv2.fillPoly(frame, [pts], (0, 165, 255))
                cv2.polylines(frame, [pts], True, (80, 210, 255), 2)
            elif obj_type == "HAZARD":
                bw = int(np.clip((0.45 * self.focal_length) / dist, 12, 240))
                bh = int(np.clip((self.known_obstacle_height * self.focal_length) / dist, 14, 240))
                cy = int(horizon + bh * 0.35)
                x1, y1 = cx - bw // 2, cy - bh // 2
                x2, y2 = cx + bw // 2, cy + bh // 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), (20, 25, 230), -1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (70, 80, 255), 2)

        return frame, horizon

    def process_frame(self, rover_pose=None, goal_cell=None):
        if rover_pose is None:
            rx, ry, ryaw = 30.0, 6.0, math.pi / 2.0
        else:
            rx, ry, ryaw = rover_pose["x_float"], rover_pose["y_float"], rover_pose["yaw"]

        z_elev, pitch, active_ramp = self.get_terrain_elevation_and_pitch(rx, ry)
        raw_frame, horizon = self._render_first_person_camera(rx, ry, ryaw, pitch, goal_cell)
        annotated = raw_frame.copy()

        hsv = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)

        # 1. Detect Red Untraversable Hazards
        mask_r1 = cv2.inRange(hsv, np.array([0, 130, 100]), np.array([8, 255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([168, 130, 100]), np.array([180, 255, 255]))
        mask_hazard = cv2.bitwise_or(mask_r1, mask_r2)

        # 2. Detect Amber Traversable Ramps
        mask_ramp = cv2.inRange(hsv, np.array([14, 150, 150]), np.array([26, 255, 255]))

        detections = []

        # Process Hazard Contours -> Only map unclipped boxes to prevent ghost obstacles!
        contours_h, _ = cv2.findContours(mask_hazard, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_h:
            area = cv2.contourArea(cnt)
            if area > 130:
                x, y, w, h = cv2.boundingRect(cnt)
                # Ignore boxes touching left/right border where width/height is clipped
                touching_edge = (x <= 3) or (x + w >= self.width - 4) or (y <= 3) or (y + h >= self.height - 4)
                cx = x + w / 2.0
                est_dist_m = (self.known_obstacle_height * self.focal_length) / float(max(h, 1))
                est_bearing_rad = -((cx - (self.width / 2.0)) / (self.width / 2.0)) * (self.fov / 2.0)

                if 0.30 <= est_dist_m <= 6.2 and not touching_edge:
                    detections.append({
                        "class_id": 0,
                        "type": "HAZARD",
                        "label": "DEBRIS_HAZARD",
                        "confidence": 0.96,
                        "bbox": [int(x), int(y), int(w), int(h)],
                        "distance_m": round(est_dist_m, 2),
                        "bearing_rad": round(est_bearing_rad, 3),
                    })
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 60, 255), 2)
                    cv2.putText(
                        annotated, f"DEBRIS {est_dist_m:.1f}m",
                        (max(4, x), max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (80, 180, 255), 1, cv2.LINE_AA
                    )

        # Process Traversable Ramp Contours
        contours_r, _ = cv2.findContours(mask_ramp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_r:
            if cv2.contourArea(cnt) > 140:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2.0
                est_dist_m = (0.22 * self.focal_length) / float(max(h, 1))
                est_bearing_rad = -((cx - (self.width / 2.0)) / (self.width / 2.0)) * (self.fov / 2.0)
                detections.append({
                    "class_id": 1,
                    "type": "RAMP",
                    "label": "CLIMB_RAMP",
                    "confidence": 0.94,
                    "bbox": [int(x), int(y), int(w), int(h)],
                    "distance_m": round(est_dist_m, 2),
                    "bearing_rad": round(est_bearing_rad, 3),
                })
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 215, 255), 2)
                cv2.putText(
                    annotated, f"TRAVERSABLE RAMP {est_dist_m:.1f}m",
                    (max(4, x), max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 215, 255), 1, cv2.LINE_AA
                )

        # HUD Overlay
        cv2.line(annotated, (self.width // 2 - 12, horizon), (self.width // 2 + 12, horizon), (0, 255, 128), 1)
        cv2.line(annotated, (self.width // 2, horizon - 10), (self.width // 2, horizon + 10), (0, 255, 128), 1)
        status_str = f"CAM 40Hz | POSE:({rx:.1f},{ry:.1f}) Z:{z_elev:.2f}m"
        if active_ramp:
            status_str += f" [CLIMBING {active_ramp}]"
        cv2.putText(annotated, status_str, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (56, 189, 248), 1, cv2.LINE_AA)

        detections.sort(key=lambda d: d["distance_m"])
        _, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 68])
        frame_b64 = base64.b64encode(buf).decode("utf-8")

        return annotated, detections, frame_b64, z_elev, pitch, active_ramp
