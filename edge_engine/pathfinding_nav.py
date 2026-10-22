import heapq
import math
import numpy as np


class AutonomousNavigator:
    def __init__(self, grid_size=60):
        self.grid_size = grid_size
        self.k_att = 1.60
        self.k_rep = 14.0
        self.rho_0 = 6.5
        self.prev_w = 0.0

    def a_star_search(self, occupancy_grid, start, goal):
        sx, sy = int(np.clip(start[0], 0, self.grid_size - 1)), int(np.clip(start[1], 0, self.grid_size - 1))
        gx, gy = int(np.clip(goal[0], 0, self.grid_size - 1)), int(np.clip(goal[1], 0, self.grid_size - 1))

        # If goal is inside an inflated obstacle cell, snap goal to nearest free cell
        if occupancy_grid[gy, gx] == 1:
            for r in range(1, 8):
                found = False
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = np.clip(gx + dx, 0, self.grid_size - 1), np.clip(gy + dy, 0, self.grid_size - 1)
                        if occupancy_grid[ny, nx] == 0:
                            gx, gy = int(nx), int(ny)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break

        open_heap = []
        heapq.heappush(open_heap, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == (gx, gy):
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            cx, cy = current
            for dx, dy, move_cost in motions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    # Allow stepping out of immediate start radius if inside inflated zone
                    if occupancy_grid[ny, nx] == 1 and math.hypot(nx - sx, ny - sy) > 2.2:
                        continue
                    tentative_g = g_score[current] + move_cost
                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        came_from[(nx, ny)] = current
                        g_score[(nx, ny)] = tentative_g
                        dx_g, dy_g = abs(gx - nx), abs(gy - ny)
                        h = (dx_g + dy_g) + (1.414 - 2.0) * min(dx_g, dy_g)
                        heapq.heappush(open_heap, (tentative_g + h, (nx, ny)))

        return [[sx, sy], [gx, gy]]

    def potential_fields_control(self, rover_pose, path, occupancy_grid, goal_cell):
        rx = rover_pose.get("x_float", float(rover_pose["x"]))
        ry = rover_pose.get("y_float", float(rover_pose["y"]))
        ryaw = rover_pose["yaw"]

        dist_to_goal = math.hypot(goal_cell[0] - rx, goal_cell[1] - ry)
        if dist_to_goal < 1.8:
            # Goal reached! Stop cleanly until operator selects a new goal
            self.prev_w = 0.0
            return 0.0, 0.0, [goal_cell[0], goal_cell[1]], True

        # Pure-Pursuit Lookahead on the A* Global Path (5 cells ahead)
        lookahead_idx = min(5, len(path) - 1)
        wx, wy = path[lookahead_idx]

        f_att_x = self.k_att * (wx - rx)
        f_att_y = self.k_att * (wy - ry)

        f_rep_x, f_rep_y = 0.0, 0.0
        obs_y_indices, obs_x_indices = np.where(occupancy_grid == 1)
        for ox, oy in zip(obs_x_indices, obs_y_indices):
            dist = math.hypot(rx - ox, ry - oy)
            if 0.3 < dist <= self.rho_0:
                rep_mag = self.k_rep * ((1.0 / dist) - (1.0 / self.rho_0)) / (dist ** 1.6)
                f_rep_x += rep_mag * ((rx - ox) / dist)
                f_rep_y += rep_mag * ((ry - oy) / dist)

        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        desired_heading = math.atan2(f_total_y, f_total_x)
        heading_error = math.atan2(math.sin(desired_heading - ryaw), math.cos(desired_heading - ryaw))

        raw_w = float(np.clip(1.6 * heading_error, -1.1, 1.1))
        angular_w = 0.65 * raw_w + 0.35 * self.prev_w
        self.prev_w = angular_w

        linear_v = float(np.clip(0.45 * max(0.25, math.cos(heading_error)), 0.14, 0.45))
        return linear_v, angular_w, [int(wx), int(wy)], False
