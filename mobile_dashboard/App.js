import React, { useEffect, useRef, useState } from "react";
import { StyleSheet, Text, View, TouchableOpacity, SafeAreaView, ScrollView, TextInput, Image } from "react-native";
import { Provider, useDispatch, useSelector } from "react-redux";
import { store, setConnectionState, ingestTelemetry, setAutonomousMode, setLocalGoal } from "./store";
import { io } from "socket.io-client";
import Svg, { Rect, Circle, Line, Polyline, G } from "react-native-svg";
import { ResponsiveContainer, LineChart, Line as ReLine, XAxis, YAxis, Tooltip, Legend } from "recharts";

const ROVER_IP = "http://localhost:4000";
const EDGE_DIRECT_API = "http://localhost:8000";
const MAP_SIZE = 280;
const GRID_DIM = 60;

function RoverDashboard() {
  const dispatch = useDispatch();
  const {
    connected, pose, active_ramp, velocity, motors, detections,
    a_star_path, scanned_obstacles, world_obstacles, world_ramps,
    goal_cell, goal_reached, camera_frame, history, autonomous
  } = useSelector((state) => state.telemetry);

  const socketRef = useRef(null);
  const mapContainerRef = useRef(null);
  const [inputX, setInputX] = useState("30");
  const [inputY, setInputY] = useState("54");

  useEffect(() => {
    const socket = io(ROVER_IP, { transports: ["websocket"] });
    socketRef.current = socket;

    socket.on("connect", () => dispatch(setConnectionState(true)));
    socket.on("disconnect", () => dispatch(setConnectionState(false)));
    socket.on("rover_telemetry_stream", (packet) => dispatch(ingestTelemetry(packet)));

    return () => socket.disconnect();
  }, [dispatch]);

  const sendCommand = async (type, payload) => {
    try {
      const endpoint = type === "SET_GOAL" ? "/goal" : "/override";
      await fetch(`${EDGE_DIRECT_API}${endpoint}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch (e) {}
    if (socketRef.current && socketRef.current.connected) {
      socketRef.current.emit("operator_command", { type, payload });
    }
  };

  const applyGoalCoordinates = (gx, gy) => {
    const parsedX = parseInt(String(gx).trim(), 10);
    const parsedY = parseInt(String(gy).trim(), 10);
    const clampedX = Math.max(2, Math.min(57, isNaN(parsedX) ? 30 : parsedX));
    const clampedY = Math.max(2, Math.min(57, isNaN(parsedY) ? 54 : parsedY));
    setInputX(String(clampedX));
    setInputY(String(clampedY));
    dispatch(setLocalGoal([clampedX, clampedY]));
    dispatch(setAutonomousMode(true));
    sendCommand("SET_GOAL", { goal_x: clampedX, goal_y: clampedY });
  };

  // Universal Web + Native Map Click Handler
  const handleMapClick = (evt) => {
    const nativeEvt = evt.nativeEvent || evt;
    let clickX = nativeEvt.offsetX ?? nativeEvt.locationX;
    let clickY = nativeEvt.offsetY ?? nativeEvt.locationY;

    if ((clickX === undefined || clickY === undefined) && evt.currentTarget && evt.currentTarget.getBoundingClientRect) {
      const rect = evt.currentTarget.getBoundingClientRect();
      clickX = (nativeEvt.clientX || 0) - rect.left;
      clickY = (nativeEvt.clientY || 0) - rect.top;
    }

    if (clickX !== undefined && clickY !== undefined) {
      const gx = Math.round((clickX / MAP_SIZE) * GRID_DIM);
      const gy = Math.round(((MAP_SIZE - clickY) / MAP_SIZE) * GRID_DIM);
      applyGoalCoordinates(gx, gy);
    }
  };

  const toCanvasX = (gx) => (gx / GRID_DIM) * MAP_SIZE;
  const toCanvasY = (gy) => MAP_SIZE - (gy / GRID_DIM) * MAP_SIZE;

  const pathPointsString = (a_star_path || [])
    .map((pt) => `${toCanvasX(pt[0]).toFixed(1)},${toCanvasY(pt[1]).toFixed(1)}`)
    .join(" ");

  const rx = pose.x_float || pose.x;
  const ry = pose.y_float || pose.y;
  const ryaw = pose.yaw || 1.57;

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView contentContainerStyle={styles.scroll}>
        <View style={styles.header}>
          <View>
            <Text style={styles.title}>MECH-MINDS // DISASTER RECONNAISSANCE ROVER</Text>
            <Text style={styles.subtitle}>TRACK 01: 40Hz V-SLAM, RAMP TRAVERSAL & DYNAMIC A* NAVIGATION</Text>
          </View>
          <View style={styles.headerRight}>
            {active_ramp && (
              <Text style={[styles.statusBadge, { backgroundColor: "#d97706", marginRight: 8 }]}>
                ▲ CLIMBING {active_ramp} (Z: {pose.z}m)
              </Text>
            )}
            <Text style={[styles.statusBadge, { backgroundColor: connected ? "#059669" : "#dc2626" }]}>
              {connected ? "● TELEMETRY ONLINE (40Hz)" : "● LINK OFFLINE"}
            </Text>
            {goal_reached && (
              <Text style={[styles.statusBadge, { backgroundColor: "#0284c7", marginLeft: 8 }]}>
                ★ GOAL REACHED
              </Text>
            )}
          </View>
        </View>

        <View style={styles.mainGrid}>
          {/* Left Panel: 40Hz Live Camera Feed */}
          <View style={styles.panelCard}>
            <View style={styles.panelHeaderRow}>
              <Text style={styles.panelTitle}>LIVE ROVER CAMERA FEED (40Hz YOLOv8n + RAMP HUD)</Text>
              <Text style={styles.cyanTag}>Objects in FOV: {detections.length}</Text>
            </View>
            <View style={styles.cameraBox}>
              {camera_frame ? (
                <Image
                  source={{ uri: `data:image/jpeg;base64,${camera_frame}` }}
                  style={styles.cameraImage}
                  resizeMode="stretch"
                />
              ) : (
                <View style={styles.cameraPlaceholder}>
                  <Text style={styles.placeholderText}>Initializing 40Hz Vision Stream...</Text>
                </View>
              )}
            </View>
            <View style={styles.detectionBar}>
              <Text style={styles.detText}>
                Nearest Scanned Object:{" "}
                {detections[0]
                  ? `${detections[0].label} at ${detections[0].distance_m}m (${(detections[0].bearing_rad * 57.3).toFixed(0)}°)`
                  : "Path Clear in Camera FOV"}
              </Text>
            </View>
          </View>

          {/* Right Panel: Interactive One-Click 2D Radar Map */}
          <View style={styles.panelCard}>
            <View style={styles.panelHeaderRow}>
              <Text style={styles.panelTitle}>2D V-SLAM SENSOR MAP & A* TRAJECTORY</Text>
              <Text style={styles.emeraldTag}>Click Any Spot on Map to Set Goal</Text>
            </View>
            <View style={styles.mapWrapper}>
              <div
                ref={mapContainerRef}
                onClick={handleMapClick}
                style={{ width: MAP_SIZE, height: MAP_SIZE, cursor: "crosshair", position: "relative" }}
              >
                <Svg width={MAP_SIZE} height={MAP_SIZE} style={styles.svgCanvas}>
                  {[10, 20, 30, 40, 50].map((g) => (
                    <G key={g}>
                      <Line x1={toCanvasX(g)} y1={0} x2={toCanvasX(g)} y2={MAP_SIZE} stroke="#1e293b" strokeWidth="1" />
                      <Line x1={0} y1={toCanvasY(g)} x2={MAP_SIZE} y2={toCanvasY(g)} stroke="#1e293b" strokeWidth="1" />
                    </G>
                  ))}

                  {/* Traversable Disaster Ramps (Amber Rectangles) */}
                  {(world_ramps || []).map((r, i) => (
                    <Rect
                      key={`ramp-${i}`}
                      x={toCanvasX(r.x - r.w / 2)}
                      y={toCanvasY(r.y + r.l / 2)}
                      width={(r.w / GRID_DIM) * MAP_SIZE}
                      height={(r.l / GRID_DIM) * MAP_SIZE}
                      fill="#f59e0b"
                      opacity={0.45}
                      stroke="#fbbf24"
                      strokeWidth="1.5"
                    />
                  ))}

                  {/* Untraversable Debris Outlines (Faint Gray) */}
                  {(world_obstacles || []).map((obs, i) => (
                    <Rect
                      key={`w-${i}`}
                      x={toCanvasX(obs[0]) - 6}
                      y={toCanvasY(obs[1]) - 6}
                      width={12}
                      height={12}
                      fill="none"
                      stroke="#475569"
                      strokeDasharray="2,2"
                    />
                  ))}

                  {/* Clean Sensed Obstacle Cells (Red) */}
                  {(scanned_obstacles || []).map((cell, idx) => (
                    <Rect
                      key={`s-${idx}`}
                      x={toCanvasX(cell[0]) - 3.5}
                      y={toCanvasY(cell[1]) - 3.5}
                      width={7}
                      height={7}
                      fill="#ef4444"
                      opacity={0.9}
                    />
                  ))}

                  {/* Exact A* Path Line */}
                  {pathPointsString.length > 0 && (
                    <Polyline
                      points={pathPointsString}
                      fill="none"
                      stroke="#38bdf8"
                      strokeWidth="3.5"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    />
                  )}

                  {/* Target Goal Beacon */}
                  <Circle cx={toCanvasX(goal_cell[0])} cy={toCanvasY(goal_cell[1])} r={9} fill="#10b981" opacity={0.3} />
                  <Circle cx={toCanvasX(goal_cell[0])} cy={toCanvasY(goal_cell[1])} r={4.5} fill="#10b981" stroke="#fff" strokeWidth="1.5" />

                  {/* Rover Position & Heading Vector */}
                  <Line
                    x1={toCanvasX(rx)}
                    y1={toCanvasY(ry)}
                    x2={toCanvasX(rx + 4.5 * Math.cos(ryaw))}
                    y2={toCanvasY(ry + 4.5 * Math.sin(ryaw))}
                    stroke="#facc15"
                    strokeWidth="2.5"
                  />
                  <Circle cx={toCanvasX(rx)} cy={toCanvasY(ry)} r={6} fill="#3b82f6" stroke="#f8fafc" strokeWidth="2" />
                </Svg>
              </div>
            </View>
            <View style={styles.mapLegendRow}>
              <Text style={styles.legendItem}>🔵 Rover ({pose.x},{pose.y})</Text>
              <Text style={styles.legendItem}>🟢 Goal ({goal_cell[0]},{goal_cell[1]})</Text>
              <Text style={styles.legendItem}>🟧 Climbable Ramp</Text>
              <Text style={styles.legendItem}>🟥 Sensed Debris</Text>
            </View>
          </View>
        </View>

        {/* Manual X, Y Coordinate Box + Ramp & Sector Presets */}
        <View style={styles.goalControlCard}>
          <Text style={styles.panelTitle}>MANUAL GOAL COORDINATES (X: 2..57, Y: 2..57) OR ONE-CLICK PRESETS</Text>
          <View style={styles.goalInputRow}>
            <View style={styles.inputGroup}>
              <Text style={styles.inputLabel}>GOAL X:</Text>
              <TextInput
                style={styles.coordInput}
                value={inputX}
                onChangeText={(txt) => setInputX(txt)}
                onSubmitEditing={() => applyGoalCoordinates(inputX, inputY)}
                keyboardType="numeric"
              />
            </View>
            <View style={styles.inputGroup}>
              <Text style={styles.inputLabel}>GOAL Y:</Text>
              <TextInput
                style={styles.coordInput}
                value={inputY}
                onChangeText={(txt) => setInputY(txt)}
                onSubmitEditing={() => applyGoalCoordinates(inputX, inputY)}
                keyboardType="numeric"
              />
            </View>
            <TouchableOpacity
              style={styles.deployGoalBtn}
              onPress={() => applyGoalCoordinates(inputX, inputY)}
            >
              <Text style={styles.deployGoalText}>SET GOAL & COMPUTE A*</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={styles.resetSimBtn}
              onPress={() => sendCommand("OVERRIDE", { autonomous_enabled: true, linear_v: 0, angular_w: 0, reset_sim: true })}
            >
              <Text style={styles.deployGoalText}>RESET ROVER TO BASE</Text>
            </TouchableOpacity>
          </View>

          <View style={styles.presetRow}>
            <TouchableOpacity style={styles.presetChip} onPress={() => applyGoalCoordinates(34, 25)}>
              <Text style={styles.presetText}>Climb Ramp Alpha (34, 25)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.presetChip} onPress={() => applyGoalCoordinates(26, 46)}>
              <Text style={styles.presetText}>Climb Ramp Bravo (26, 46)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.presetChip} onPress={() => applyGoalCoordinates(30, 55)}>
              <Text style={styles.presetText}>North Disaster Zone (30, 55)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.presetChip} onPress={() => applyGoalCoordinates(30, 6)}>
              <Text style={styles.presetText}>Return to Base (30, 6)</Text>
            </TouchableOpacity>
          </View>
        </View>

        <View style={styles.metricsRow}>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>3D POSE (X, Y, Z-ELEV)</Text>
            <Text style={styles.metricVal}>
              ({pose.x_float ?? pose.x}, {pose.y_float ?? pose.y}, {pose.z ?? 0.05}m)
            </Text>
          </View>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>CHASSIS PITCH / YAW</Text>
            <Text style={styles.metricVal}>
              Pitch: {((pose.pitch || 0) * 57.3).toFixed(1)}° | Yaw: {((pose.yaw || 0) * 57.3).toFixed(0)}°
            </Text>
          </View>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>L298N MOTOR PWM (L / R)</Text>
            <Text style={styles.metricVal}>
              L: {Number(motors?.pwm_l || 0).toFixed(1)} | R: {Number(motors?.pwm_r || 0).toFixed(1)}
            </Text>
          </View>
        </View>

        <View style={styles.goalControlCard}>
          <Text style={styles.panelTitle}>REAL-TIME C++17 PID MOTOR ACTUATION TELEMETRY</Text>
          <View style={{ width: "100%", height: 140, marginTop: 6 }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={history}>
                <XAxis dataKey="time" stroke="#64748b" />
                <YAxis stroke="#64748b" />
                <Tooltip contentStyle={{ backgroundColor: "#0f172a", borderColor: "#334155" }} />
                <Legend />
                <ReLine type="monotone" dataKey="pwmL" name="Left Motor PWM" stroke="#38bdf8" strokeWidth={2} dot={false} isAnimationActive={false} />
                <ReLine type="monotone" dataKey="pwmR" name="Right Motor PWM" stroke="#f43f5e" strokeWidth={2} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </View>
        </View>

        <View style={styles.controlsRow}>
          <TouchableOpacity
            style={[styles.btn, { backgroundColor: autonomous ? "#2563eb" : "#475569" }]}
            onPress={() => {
              const next = !autonomous;
              dispatch(setAutonomousMode(next));
              sendCommand("OVERRIDE", { autonomous_enabled: next, linear_v: 0, angular_w: 0 });
            }}
          >
            <Text style={styles.btnText}>
              {autonomous ? "● AUTONOMOUS NAVIGATION ENGAGED (A* + APF)" : "● MANUAL STANDBY MODE"}
            </Text>
          </TouchableOpacity>

          <TouchableOpacity
            style={[styles.btn, styles.estopBtn]}
            onPress={() => {
              dispatch(setAutonomousMode(false));
              sendCommand("OVERRIDE", { autonomous_enabled: false, linear_v: 0, angular_w: 0 });
            }}
          >
            <Text style={styles.btnText}>■ HARDWARE EMERGENCY STOP</Text>
          </TouchableOpacity>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

export default function App() {
  return (
    <Provider store={store}>
      <RoverDashboard />
    </Provider>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: "#060b16" },
  scroll: { padding: 16, maxWidth: 1200, alignSelf: "center", width: "100%" },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    backgroundColor: "#0f172a",
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "#1e293b",
    marginBottom: 14,
    flexWrap: "wrap",
    gap: 8,
  },
  title: { color: "#f8fafc", fontSize: 18, fontWeight: "800", letterSpacing: 0.8 },
  subtitle: { color: "#38bdf8", fontSize: 11, fontWeight: "600", marginTop: 2 },
  headerRight: { flexDirection: "row", alignItems: "center", flexWrap: "wrap" },
  statusBadge: { color: "#fff", fontSize: 11, fontWeight: "700", paddingHorizontal: 12, paddingVertical: 6, borderRadius: 20 },
  mainGrid: { flexDirection: "row", flexWrap: "wrap", gap: 14, marginBottom: 14 },
  panelCard: {
    flex: 1,
    minWidth: 320,
    backgroundColor: "#0f172a",
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "#1e293b",
  },
  panelHeaderRow: { flexDirection: "row", justifyContent: "space-between", alignItems: "center", marginBottom: 10 },
  panelTitle: { color: "#94a3b8", fontSize: 12, fontWeight: "700", letterSpacing: 0.5 },
  cyanTag: { color: "#38bdf8", fontSize: 11, fontWeight: "700", backgroundColor: "#0c4a6e", paddingHorizontal: 8, paddingVertical: 3, borderRadius: 6 },
  emeraldTag: { color: "#34d399", fontSize: 11, fontWeight: "700", backgroundColor: "#064e3b", paddingHorizontal: 8, paddingVertical: 3, borderRadius: 6 },
  cameraBox: { height: 260, backgroundColor: "#020617", borderRadius: 8, overflow: "hidden", borderWidth: 1, borderColor: "#1e293b" },
  cameraImage: { width: "100%", height: "100%" },
  cameraPlaceholder: { flex: 1, justifyContent: "center", alignItems: "center" },
  placeholderText: { color: "#64748b", fontSize: 13 },
  detectionBar: { marginTop: 10, backgroundColor: "#1e293b", padding: 8, borderRadius: 6 },
  detText: { color: "#38bdf8", fontSize: 12, fontWeight: "600", textAlign: "center" },
  mapWrapper: { alignItems: "center", justifyContent: "center", backgroundColor: "#020617", padding: 8, borderRadius: 8, borderWidth: 1, borderColor: "#1e293b" },
  svgCanvas: { backgroundColor: "#090f1f", borderRadius: 6 },
  mapLegendRow: { flexDirection: "row", justifyContent: "space-around", marginTop: 10, flexWrap: "wrap" },
  legendItem: { color: "#cbd5e1", fontSize: 11, fontWeight: "600" },
  goalControlCard: { backgroundColor: "#0f172a", padding: 14, borderRadius: 12, borderWidth: 1, borderColor: "#1e293b", marginBottom: 14 },
  goalInputRow: { flexDirection: "row", alignItems: "center", gap: 10, marginTop: 10, flexWrap: "wrap" },
  inputGroup: { flexDirection: "row", alignItems: "center", backgroundColor: "#1e293b", paddingHorizontal: 10, paddingVertical: 6, borderRadius: 8 },
  inputLabel: { color: "#94a3b8", fontSize: 12, fontWeight: "700", marginRight: 6 },
  coordInput: { color: "#f8fafc", fontSize: 14, fontWeight: "700", width: 50, textAlign: "center", backgroundColor: "#0f172a", paddingVertical: 4, borderRadius: 4 },
  deployGoalBtn: { backgroundColor: "#059669", paddingHorizontal: 16, paddingVertical: 10, borderRadius: 8 },
  resetSimBtn: { backgroundColor: "#475569", paddingHorizontal: 14, paddingVertical: 10, borderRadius: 8 },
  deployGoalText: { color: "#fff", fontSize: 12, fontWeight: "700" },
  presetRow: { flexDirection: "row", gap: 8, marginTop: 10, flexWrap: "wrap" },
  presetChip: { flex: 1, minWidth: 140, backgroundColor: "#1e293b", paddingVertical: 8, paddingHorizontal: 10, borderRadius: 6, alignItems: "center", borderWidth: 1, borderColor: "#334155" },
  presetText: { color: "#fbbf24", fontSize: 11, fontWeight: "700" },
  metricsRow: { flexDirection: "row", gap: 10, marginBottom: 14, flexWrap: "wrap" },
  metricBox: { flex: 1, minWidth: 180, backgroundColor: "#0f172a", padding: 12, borderRadius: 10, borderWidth: 1, borderColor: "#1e293b" },
  metricLabel: { color: "#64748b", fontSize: 11, fontWeight: "700" },
  metricVal: { color: "#f8fafc", fontSize: 15, fontWeight: "800", marginTop: 4 },
  controlsRow: { flexDirection: "row", gap: 12 },
  btn: { flex: 1, paddingVertical: 14, borderRadius: 10, alignItems: "center" },
  estopBtn: { backgroundColor: "#dc2626" },
  btnText: { color: "#ffffff", fontWeight: "800", fontSize: 13 },
});
