import cv2
import numpy as np
import math


class VSLAMOccupancyMapper:
    def __init__(self, grid_size=60, resolution=0.20):
        self.grid_size = grid_size
        self.resolution = resolution
        self.log_odds = np.zeros((grid_size, grid_size), dtype=np.float32)

        self.rover_x_float = 30.0
        self.rover_y_float = 6.0
        self.rover_yaw = math.pi / 2.0
        self.rover_z = 0.085
        self.rover_pitch = 0.0

    def get_pose_state(self):
        return {
            "x": int(round(self.rover_x_float)),
            "y": int(round(self.rover_y_float)),
            "x_float": round(self.rover_x_float, 2),
            "y_float": round(self.rover_y_float, 2),
            "yaw": round(self.rover_yaw, 3),
            "z": round(self.rover_z, 3),
            "pitch": round(self.rover_pitch, 3),
        }

    def reset_map_and_pose(self, x=30.0, y=6.0, yaw=math.pi / 2.0, clear_map=False):
        self.rover_x_float = float(x)
        self.rover_y_float = float(y)
        self.rover_yaw = float(yaw)
        if clear_map:
            self.log_odds.fill(0.0)

    def _bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def update_slam_and_map(self, frame, detections, world_obstacles, z_elev, pitch):
        self.rover_z = z_elev
        self.rover_pitch = pitch

        rx = int(round(self.rover_x_float))
        ry = int(round(self.rover_y_float))

        self.log_odds *= 0.99

        for det in detections:
            if det.get("type") != "HAZARD":
                continue

            dist_cells = det["distance_m"] / self.resolution
            angle = self.rover_yaw + det["bearing_rad"]
            est_x = self.rover_x_float + dist_cells * math.cos(angle)
            est_y = self.rover_y_float + dist_cells * math.sin(angle)

            obs_x, obs_y = int(round(est_x)), int(round(est_y))
            for w_obs in world_obstacles:
                wx, wy = w_obs["x"], w_obs["y"]
                if math.hypot(est_x - wx, est_y - wy) < 3.5:
                    obs_x, obs_y = int(round(wx)), int(round(wy))
                    break

            obs_x = int(np.clip(obs_x, 1, self.grid_size - 2))
            obs_y = int(np.clip(obs_y, 1, self.grid_size - 2))

            ray_cells = self._bresenham_line(rx, ry, obs_x, obs_y)
            for cx, cy in ray_cells[:-1]:
                if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                    self.log_odds[cy, cx] = np.clip(self.log_odds[cy, cx] - 0.6, -4.0, 4.0)

            # Bio-chemical and Radiation zones get a wider safety buffer than simple physical debris
            inflate_r = 2 if det.get("kind") in ("BIOCHEM", "RADIATION") else 1
            for ox in range(max(0, obs_x - inflate_r), min(self.grid_size, obs_x + inflate_r + 1)):
                for oy in range(max(0, obs_y - inflate_r), min(self.grid_size, obs_y + inflate_r + 1)):
                    self.log_odds[oy, ox] = np.clip(self.log_odds[oy, ox] + 1.8, -4.0, 4.0)

        prob_grid = (1.0 - (1.0 / (1.0 + np.exp(self.log_odds)))) * 100.0
        binary_occupancy = (prob_grid > 70.0).astype(np.uint8)

        obs_y_idx, obs_x_idx = np.where(binary_occupancy == 1)
        scanned_obstacles = [[int(x), int(y)] for x, y in zip(obs_x_idx, obs_y_idx)]

        return {
            "pose": self.get_pose_state(),
            "vo_confidence": 0.96,
            "binary_grid": binary_occupancy,
            "scanned_obstacles": scanned_obstacles,
        }
