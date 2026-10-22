import { configureStore, createSlice } from "@reduxjs/toolkit";

const telemetrySlice = createSlice({
  name: "telemetry",
  initialState: {
    connected: false,
    pose: { x: 30, y: 6, x_float: 30.0, y_float: 6.0, yaw: 1.57, z: 0.05, pitch: 0.0 },
    active_ramp: null,
    velocity: { linear: 0, angular: 0 },
    motors: { pwm_l: 0, pwm_r: 0, dir_l: "FWD", dir_r: "FWD" },
    detections: [],
    a_star_path: [],
    scanned_obstacles: [],
    world_obstacles: [],
    world_ramps: [],
    goal_cell: [30, 54],
    goal_reached: false,
    camera_frame: null,
    history: [],
    autonomous: true,
  },
  reducers: {
    setConnectionState(state, action) {
      state.connected = action.payload;
    },
    ingestTelemetry(state, action) {
      const data = action.payload;
      state.pose = data.pose || state.pose;
      state.active_ramp = data.active_ramp || null;
      state.velocity = data.velocity || state.velocity;
      state.motors = data.motors || state.motors;
      state.detections = data.detections || [];
      state.a_star_path = data.a_star_path || [];
      state.scanned_obstacles = data.scanned_obstacles || [];
      state.world_obstacles = data.world_obstacles || [];
      state.world_ramps = data.world_ramps || [];
      if (data.goal_cell) state.goal_cell = data.goal_cell;
      state.goal_reached = Boolean(data.goal_reached);
      if (data.camera_frame) state.camera_frame = data.camera_frame;
      state.history = [
        ...state.history.slice(-30),
        {
          time: new Date().toLocaleTimeString().slice(3, 8),
          pwmL: Number((data.motors?.pwm_l || 0).toFixed(1)),
          pwmR: Number((data.motors?.pwm_r || 0).toFixed(1)),
        },
      ];
    },
    setAutonomousMode(state, action) {
      state.autonomous = action.payload;
    },
    setLocalGoal(state, action) {
      state.goal_cell = action.payload;
    },
  },
});

export const { setConnectionState, ingestTelemetry, setAutonomousMode, setLocalGoal } = telemetrySlice.actions;
export const store = configureStore({ reducer: { telemetry: telemetrySlice.reducer } });
