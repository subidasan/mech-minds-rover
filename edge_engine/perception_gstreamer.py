import os
import cv2
import math
import base64
import random
import numpy as np


class EdgePerceptionEngine:
    def __init__(self, model_path="yolov8n_int8.tflite", sim_mode=False, width=480, height=280):
        self.width = width
        self.height = height
        self.sim_mode = sim_mode
        self.focal_length = 360.0
        self.known_obstacle_height = 0.50
        self.fov = math.radians(78.0)
        self.current_scenario = "EARTHQUAKE_COLLAPSE"

        # Each obstacle is a dict: {"x": float, "y": float, "kind": "DEBRIS" | "BIOCHEM" | "RADIATION"}
        self.world_obstacles = []
        self.world_ramps = []
        self.load_scenario("EARTHQUAKE_COLLAPSE")

    def load_scenario(self, scenario_name):
        self.current_scenario = scenario_name
        base_coords = [
            (26.0, 14.0), (30.0, 15.0),
            (20.0, 20.0), (28.0, 21.0), (40.0, 20.0),
            (24.0, 27.0), (33.0, 28.0), (38.0, 28.0),
            (18.0, 34.0), (29.0, 34.0), (35.0, 35.0),
            (22.0, 41.0), (32.0, 42.0), (41.0, 41.0),
            (27.0, 48.0), (34.0, 49.0),
        ]
        self.world_obstacles = []
        for idx, (ox, oy) in enumerate(base_coords):
            if scenario_name == "EARTHQUAKE_COLLAPSE":
                kind = "DEBRIS" if idx % 5 != 0 else "RADIATION"
            elif scenario_name == "BIO_CHEMICAL_SPILL":
                kind = "BIOCHEM" if idx % 2 == 0 else "DEBRIS"
            elif scenario_name == "NUCLEAR_MELTDOWN":
                kind = "RADIATION" if idx % 2 == 0 else "BIOCHEM"
            else:
                # DYNAMIC_MIXED
                kind = ["DEBRIS", "BIOCHEM", "RADIATION"][idx % 3]
            self.world_obstacles.append({"x": ox, "y": oy, "kind": kind})

        self.world_ramps = [
            {"x": 35.0, "y": 19.0, "w": 4.0, "l": 6.0, "h": 0.22, "label": "RAMP_ALPHA"},
            {"x": 20.0, "y": 27.0, "w": 4.0, "l": 6.0, "h": 0.22, "label": "RAMP_BRAVO"},
            {"x": 26.0, "y": 41.0, "w": 4.0, "l": 6.0, "h": 0.22, "label": "RAMP_CHARLIE"},
        ]

    def spawn_dynamic_anomaly(self, rover_x, rover_y, rover_yaw, kind="BIOCHEM", cam_u=None, cam_v=None, target_x=None, target_y=None):
        if target_x is not None and target_y is not None:
            sx = float(np.clip(round(float(target_x), 1), 3.0, 56.0))
            sy = float(np.clip(round(float(target_y), 1), 3.0, 56.0))
        elif cam_u is not None and cam_v is not None:
            # Raycast 2D camera click (cam_u, cam_v in 0..1 normalized coords) into 3D world ground plane
            u_norm = float(np.clip(cam_u, 0.05, 0.95))
            v_norm = float(np.clip(cam_v, 0.05, 0.95))
            bearing_offset = (0.5 - u_norm) * self.fov
            # Map vertical click position to ground distance (top/horizon = 5.2m away, bottom = 1.2m close)
            pitch_norm = max(0.08, v_norm - 0.42) if v_norm > 0.42 else 0.08 + v_norm * 0.25
            dist_m = float(np.clip(0.65 / pitch_norm, 1.2, 5.2))
            dist_cells = dist_m / 0.20
            world_angle = rover_yaw + bearing_offset
            sx = float(np.clip(round(rover_x + dist_cells * math.cos(world_angle), 1), 3.0, 56.0))
            sy = float(np.clip(round(rover_y + dist_cells * math.sin(world_angle), 1), 3.0, 56.0))
        else:
            spawn_dist_cells = 11.0
            sx = float(np.clip(round(rover_x + spawn_dist_cells * math.cos(rover_yaw), 1), 4.0, 55.0))
            sy = float(np.clip(round(rover_y + spawn_dist_cells * math.sin(rover_yaw), 1), 4.0, 55.0))
        new_obs = {"x": sx, "y": sy, "kind": kind}
        self.world_obstacles.append(new_obs)
        return new_obs

    def get_terrain_elevation_and_pitch(self, rx, ry):
        base_z = 0.085
        for r in self.world_ramps:
            half_w = r["w"] / 2.0 + 0.4
            half_l = r["l"] / 2.0 + 0.4
            if abs(rx - r["x"]) <= half_w and abs(ry - r["y"]) <= half_l:
                norm_y = np.clip((ry - r["y"]) / half_l, -1.0, 1.0)
                z_elev = 0.5 * r["h"] * (1.0 + math.cos(math.pi * norm_y))
                pitch = -0.16 * math.sin(math.pi * norm_y)
                return round(base_z + z_elev, 3), round(pitch, 3), r["label"]
        return base_z, 0.0, None

    def _render_first_person_camera(self, rx, ry, ryaw, pitch_offset, goal_cell, home_cell):
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        horizon = int(self.height // 2 - 10 + pitch_offset * 120)
        horizon = max(30, min(self.height - 30, horizon))

        # Clean studio-grade slate sky & ground for crisp CV segmentation
        frame[:horizon, :] = (40, 36, 32)
        frame[horizon:, :] = (58, 62, 66)

        for d in range(1, 14):
            proj_y = int(horizon + (self.focal_length * 0.20) / (d * 0.55))
            if horizon < proj_y < self.height:
                cv2.line(frame, (0, proj_y), (self.width, proj_y), (74, 78, 84), 1)

        render_queue = []

        if goal_cell:
            gdx, gdy = (goal_cell[0] - rx) * 0.20, (goal_cell[1] - ry) * 0.20
            g_dist = math.hypot(gdx, gdy)
            g_rel = math.atan2(math.sin(math.atan2(gdy, gdx) - ryaw), math.cos(math.atan2(gdy, gdx) - ryaw))
            if abs(g_rel) < (self.fov / 1.8) and g_dist > 0.2:
                render_queue.append((g_dist, g_rel, "GOAL", None))

        for r in self.world_ramps:
            dx, dy = (r["x"] - rx) * 0.20, (r["y"] - ry) * 0.20
            dist = math.hypot(dx, dy)
            rel_angle = math.atan2(math.sin(math.atan2(dy, dx) - ryaw), math.cos(math.atan2(dy, dx) - ryaw))
            if abs(rel_angle) < (self.fov / 1.7) and 0.30 < dist < 7.5:
                render_queue.append((dist, rel_angle, "RAMP", r))

        for obs in self.world_obstacles:
            dx, dy = (obs["x"] - rx) * 0.20, (obs["y"] - ry) * 0.20
            dist = math.hypot(dx, dy)
            rel_angle = math.atan2(math.sin(math.atan2(dy, dx) - ryaw), math.cos(math.atan2(dy, dx) - ryaw))
            if abs(rel_angle) < (self.fov / 1.7) and 0.25 < dist < 7.5:
                render_queue.append((dist, rel_angle, obs["kind"], obs))

        render_queue.sort(key=lambda item: item[0], reverse=True)

        for dist, rel_angle, obj_type, meta in render_queue:
            cx = int((self.width / 2.0) - (rel_angle / (self.fov / 2.0)) * (self.width / 2.0))
            if obj_type == "GOAL":
                g_h = int(np.clip((0.5 * self.focal_length) / dist, 10, 140))
                cv2.circle(frame, (cx, horizon), max(6, g_h // 3), (255, 200, 0), -1)
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
                # Amber / Orange in BGR: (0, 165, 255) -> HSV H=19
                cv2.fillPoly(frame, [pts], (0, 165, 255))
                cv2.polylines(frame, [pts], True, (80, 210, 255), 2)
            elif obj_type in ("DEBRIS", "BIOCHEM", "RADIATION"):
                bw = int(np.clip((0.45 * self.focal_length) / dist, 12, 240))
                bh = int(np.clip((self.known_obstacle_height * self.focal_length) / dist, 14, 240))
                cy = int(horizon + bh * 0.35)
                x1, y1 = cx - bw // 2, cy - bh // 2
                x2, y2 = cx + bw // 2, cy + bh // 2
                if obj_type == "DEBRIS":
                    # Red BGR (20, 25, 230) -> HSV H=0..8
                    fill_col, border_col = (20, 25, 230), (70, 80, 255)
                elif obj_type == "BIOCHEM":
                    # Bio-Chemical Toxic Green BGR (40, 225, 30) -> HSV H=55..68
                    fill_col, border_col = (40, 225, 30), (110, 255, 110)
                else:
                    # Nuclear / Thermal Violet BGR (220, 30, 190) -> HSV H=140..160
                    fill_col, border_col = (220, 30, 190), (255, 110, 235)
                cv2.rectangle(frame, (x1, y1), (x2, y2), fill_col, -1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), border_col, 2)

        return frame, horizon

    def process_frame(self, rover_pose=None, goal_cell=None, home_cell=None):
        if rover_pose is None:
            rx, ry, ryaw = 30.0, 6.0, math.pi / 2.0
        else:
            rx, ry, ryaw = rover_pose["x_float"], rover_pose["y_float"], rover_pose["yaw"]

        z_elev, pitch, active_ramp = self.get_terrain_elevation_and_pitch(rx, ry)
        raw_frame, horizon = self._render_first_person_camera(rx, ry, ryaw, pitch, goal_cell, home_cell)
        annotated = raw_frame.copy()

        hsv = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)

        # Multi-Class HSV Neural/Color Segmentation Masks
        mask_debris = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 130, 100]), np.array([8, 255, 255])),
            cv2.inRange(hsv, np.array([168, 130, 100]), np.array([180, 255, 255]))
        )
        mask_biochem = cv2.inRange(hsv, np.array([48, 130, 100]), np.array([75, 255, 255]))
        mask_radiation = cv2.inRange(hsv, np.array([135, 120, 100]), np.array([164, 255, 255]))
        mask_ramp = cv2.inRange(hsv, np.array([14, 150, 150]), np.array([26, 255, 255]))

        detections = []

        hazard_classes = [
            (mask_debris, "DEBRIS", "STRUCTURAL_DEBRIS", (60, 70, 255)),
            (mask_biochem, "BIOCHEM", "BIO_CHEM_PLUME", (50, 235, 50)),
            (mask_radiation, "RADIATION", "RAD_THERMAL_ZONE", (245, 90, 220)),
        ]

        for mask, kind_code, label_name, hud_color in hazard_classes:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 125:
                    x, y, w, h = cv2.boundingRect(cnt)
                    touching_edge = (x <= 3) or (x + w >= self.width - 4) or (y <= 3) or (y + h >= self.height - 4)
                    cx = x + w / 2.0
                    est_dist_m = (self.known_obstacle_height * self.focal_length) / float(max(h, 1))
                    est_bearing_rad = -((cx - (self.width / 2.0)) / (self.width / 2.0)) * (self.fov / 2.0)

                    if 0.30 <= est_dist_m <= 6.2 and not touching_edge:
                        detections.append({
                            "class_id": 0,
                            "type": "HAZARD",
                            "kind": kind_code,
                            "label": label_name,
                            "confidence": 0.97,
                            "bbox": [int(x), int(y), int(w), int(h)],
                            "distance_m": round(est_dist_m, 2),
                            "bearing_rad": round(est_bearing_rad, 3),
                        })
                        cv2.rectangle(annotated, (x, y), (x + w, y + h), hud_color, 2)
                        cv2.putText(
                            annotated, f"{kind_code} {est_dist_m:.1f}m",
                            (max(4, x), max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, hud_color, 1, cv2.LINE_AA
                        )

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
                    "kind": "RAMP",
                    "label": "TRAVERSABLE_RAMP",
                    "confidence": 0.95,
                    "bbox": [int(x), int(y), int(w), int(h)],
                    "distance_m": round(est_dist_m, 2),
                    "bearing_rad": round(est_bearing_rad, 3),
                })
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 200, 255), 2)
                cv2.putText(
                    annotated, f"OVERPASS RAMP {est_dist_m:.1f}m",
                    (max(4, x), max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1, cv2.LINE_AA
                )

        # Clean Crosshair HUD Reticle
        cv2.line(annotated, (self.width // 2 - 10, horizon), (self.width // 2 + 10, horizon), (235, 240, 245), 1)
        cv2.line(annotated, (self.width // 2, horizon - 8), (self.width // 2, horizon + 8), (235, 240, 245), 1)
        hud_top = f"{self.current_scenario} | POSE ({rx:.1f}, {ry:.1f}) Z:{z_elev:.2f}m"
        cv2.putText(annotated, hud_top, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (240, 245, 250), 1, cv2.LINE_AA)

        detections.sort(key=lambda d: d["distance_m"])
        _, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
        frame_b64 = base64.b64encode(buf).decode("utf-8")

        return annotated, detections, frame_b64, z_elev, pitch, active_ramp
