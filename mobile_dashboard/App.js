import React, { useEffect, useRef, useState } from "react";
import { StyleSheet, Text, View, TouchableOpacity, SafeAreaView, ScrollView, TextInput, Image } from "react-native";
import { Provider, useDispatch, useSelector } from "react-redux";
import { store, setConnectionState, ingestTelemetry, setAutonomousMode, setLocalGoal, setLocalHome } from "./store";
import Svg, { Rect, Circle, Line, Polyline, G } from "react-native-svg";
import { ResponsiveContainer, LineChart, Line as ReLine, XAxis, YAxis, Tooltip } from "recharts";

const EDGE_DIRECT_API = "http://localhost:8000";
const MAP_SIZE = 300;
const GRID_DIM = 60;

const SCENARIOS = [
  { id: "EARTHQUAKE_COLLAPSE", label: "Seismic Collapse" },
  { id: "BIO_CHEMICAL_SPILL", label: "Bio-Chem Spill" },
  { id: "NUCLEAR_MELTDOWN", label: "Radiation Zone" },
  { id: "DYNAMIC_MIXED", label: "Multi-Hazard Mix" },
];

function RoverDashboard() {
  const dispatch = useDispatch();
  const {
    connected, scenario, pose, home_cell, active_ramp, drive_mode, safety_score,
    learned_zones, learned_counts, mission_log, velocity, motors, detections,
    a_star_path, scanned_obstacles, world_obstacles, world_ramps,
    goal_cell, goal_reached, camera_frame, history, autonomous
  } = useSelector((state) => state.telemetry);

  const mapContainerRef = useRef(null);
  const [inputX, setInputX] = useState("30");
  const [inputY, setInputY] = useState("54");
  const [homeX, setHomeX] = useState("30");
  const [homeY, setHomeY] = useState("6");
  const [clickMode, setClickMode] = useState("GOAL"); // "GOAL" or "HOME"

  useEffect(() => {
    let isMounted = true;
    const pollTelemetry = async () => {
      try {
        const res = await fetch(`${EDGE_DIRECT_API}/status`);
        if (res.ok) {
          const data = await res.json();
          if (isMounted && data && data.pose) {
            dispatch(ingestTelemetry(data));
          }
        } else if (isMounted) {
          dispatch(setConnectionState(false));
        }
      } catch (err) {
        if (isMounted) dispatch(setConnectionState(false));
      }
    };

    const intervalId = setInterval(pollTelemetry, 90);
    pollTelemetry();
    return () => {
      isMounted = false;
      clearInterval(intervalId);
    };
  }, [dispatch]);

  const postAPI = async (endpoint, payload) => {
    try {
      await fetch(`${EDGE_DIRECT_API}${endpoint}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch (e) {}
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
    postAPI("/goal", { goal_x: clampedX, goal_y: clampedY });
  };

  const applyHomeCoordinates = (hx, hy, useCurrent = false) => {
    if (useCurrent) {
      setHomeX(String(pose.x));
      setHomeY(String(pose.y));
      dispatch(setLocalHome([pose.x, pose.y]));
      postAPI("/home", { home_x: pose.x, home_y: pose.y, use_current_pose: true });
      return;
    }
    const parsedX = parseInt(String(hx).trim(), 10);
    const parsedY = parseInt(String(hy).trim(), 10);
    const clampedX = Math.max(2, Math.min(57, isNaN(parsedX) ? 30 : parsedX));
    const clampedY = Math.max(2, Math.min(57, isNaN(parsedY) ? 6 : parsedY));
    setHomeX(String(clampedX));
    setHomeY(String(clampedY));
    dispatch(setLocalHome([clampedX, clampedY]));
    postAPI("/home", { home_x: clampedX, home_y: clampedY, use_current_pose: false });
  };

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
      if (clickMode === "HOME") {
        applyHomeCoordinates(gx, gy, false);
        setClickMode("GOAL");
      } else {
        applyGoalCoordinates(gx, gy);
      }
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

  const getObstacleColor = (kind) => {
    if (kind === "BIOCHEM") return "#34C759";
    if (kind === "RADIATION") return "#AF52DE";
    return "#FF3B30";
  };

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView contentContainerStyle={styles.scroll}>
        {/* Apple Minimalist Top Navigation Bar */}
        <View style={styles.navBar}>
          <View>
            <Text style={styles.brandSubtitle}>MECH-MINDS AUTONOMOUS SYSTEMS</Text>
            <Text style={styles.brandTitle}>Reconnaissance Rover OS</Text>
          </View>

          {/* Segmented Disaster Environment Switcher */}
          <View style={styles.segmentedControl}>
            {SCENARIOS.map((sc) => {
              const active = scenario === sc.id;
              return (
                <TouchableOpacity
                  key={sc.id}
                  style={[styles.segmentBtn, active && styles.segmentBtnActive]}
                  onPress={() => postAPI("/scenario", { scenario: sc.id })}
                >
                  <Text style={[styles.segmentText, active && styles.segmentTextActive]}>
                    {sc.label}
                  </Text>
                </TouchableOpacity>
              );
            })}
          </View>

          <View style={styles.statusRow}>
            <View style={[styles.pillBadge, { backgroundColor: connected ? "#E8F8EE" : "#FFEBEA" }]}>
              <View style={[styles.dot, { backgroundColor: connected ? "#34C759" : "#FF3B30" }]} />
              <Text style={[styles.pillText, { color: connected ? "#1D7A36" : "#C92A2A" }]}>
                {connected ? "40Hz Neural Link" : "Offline"}
              </Text>
            </View>
          </View>
        </View>

        {/* Primary Bento Row: Vision Feed + Spatial V-SLAM Map */}
        <View style={styles.bentoGrid}>
          {/* Left Card: Live Vision Feed & Multi-Hazard Neural Classifier */}
          <View style={styles.bentoCard}>
            <View style={styles.cardHeader}>
              <View>
                <Text style={styles.cardEyebrow}>OPTICAL PERCEPTION</Text>
                <Text style={styles.cardTitle}>Multi-Hazard Vision Feed</Text>
              </View>
              <View style={styles.subtleChip}>
                <Text style={styles.subtleChipText}>{drive_mode}</Text>
              </View>
            </View>

            <View style={styles.cameraContainer}>
              {camera_frame ? (
                <Image
                  source={{ uri: `data:image/jpeg;base64,${camera_frame}` }}
                  style={styles.cameraImage}
                  resizeMode="stretch"
                />
              ) : (
                <View style={styles.cameraEmpty}>
                  <Text style={styles.emptyText}>Awaiting Optical Stream...</Text>
                </View>
              )}
            </View>

            {/* Dynamic Unmapped Anomaly Spawner (Proves Real-Time Learning in Any Disaster) */}
            <View style={styles.anomalyBar}>
              <Text style={styles.microLabel}>INJECT UNMAPPED ANOMALY AHEAD:</Text>
              <View style={styles.anomalyBtnGroup}>
                <TouchableOpacity
                  style={[styles.anomalyChip, { borderColor: "#FF3B30" }]}
                  onPress={() => postAPI("/scenario", { spawn_anomaly_kind: "DEBRIS" })}
                >
                  <Text style={[styles.anomalyChipText, { color: "#FF3B30" }]}>+ Seismic Debris</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={[styles.anomalyChip, { borderColor: "#34C759" }]}
                  onPress={() => postAPI("/scenario", { spawn_anomaly_kind: "BIOCHEM" })}
                >
                  <Text style={[styles.anomalyChipText, { color: "#248A3D" }]}>+ Bio-Chem Plume</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={[styles.anomalyChip, { borderColor: "#AF52DE" }]}
                  onPress={() => postAPI("/scenario", { spawn_anomaly_kind: "RADIATION" })}
                >
                  <Text style={[styles.anomalyChipText, { color: "#8944AB" }]}>+ Radiation Pocket</Text>
                </TouchableOpacity>
              </View>
            </View>
          </View>

          {/* Right Card: Interactive Spatial Map (One-Click Goal & Home Base) */}
          <View style={styles.bentoCard}>
            <View style={styles.cardHeader}>
              <View>
                <Text style={styles.cardEyebrow}>SPATIAL TOPOLOGY & A* ROUTING</Text>
                <Text style={styles.cardTitle}>Lifelong V-SLAM Map</Text>
              </View>
              <View style={styles.mapModeToggle}>
                <TouchableOpacity
                  style={[styles.modePill, clickMode === "GOAL" && styles.modePillActive]}
                  onPress={() => setClickMode("GOAL")}
                >
                  <Text style={[styles.modePillText, clickMode === "GOAL" && styles.modePillTextActive]}>
                    Click sets Goal
                  </Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={[styles.modePill, clickMode === "HOME" && styles.modePillActiveHome]}
                  onPress={() => setClickMode("HOME")}
                >
                  <Text style={[styles.modePillText, clickMode === "HOME" && styles.modePillTextActive]}>
                    Click sets Home
                  </Text>
                </TouchableOpacity>
              </View>
            </View>

            <View style={styles.mapCanvasContainer}>
              <div
                ref={mapContainerRef}
                onClick={handleMapClick}
                style={{ width: MAP_SIZE, height: MAP_SIZE, cursor: "crosshair", position: "relative" }}
              >
                <Svg width={MAP_SIZE} height={MAP_SIZE} style={styles.svgMap}>
                  {[10, 20, 30, 40, 50].map((g) => (
                    <G key={g}>
                      <Line x1={toCanvasX(g)} y1={0} x2={toCanvasX(g)} y2={MAP_SIZE} stroke="#E5E5EA" strokeWidth="1" />
                      <Line x1={0} y1={toCanvasY(g)} x2={MAP_SIZE} y2={toCanvasY(g)} stroke="#E5E5EA" strokeWidth="1" />
                    </G>
                  ))}

                  {/* Traversable Overpass Ramps (Warm Amber) */}
                  {(world_ramps || []).map((r, i) => (
                    <Rect
                      key={`ramp-${i}`}
                      x={toCanvasX(r.x - r.w / 2)}
                      y={toCanvasY(r.y + r.l / 2)}
                      width={(r.w / GRID_DIM) * MAP_SIZE}
                      height={(r.l / GRID_DIM) * MAP_SIZE}
                      fill="#FF9500"
                      opacity={0.32}
                      stroke="#FF9500"
                      strokeWidth="1.5"
                      rx="3"
                    />
                  ))}

                  {/* Multi-Disaster Arena Obstacles (Color-coded by class) */}
                  {(world_obstacles || []).map((obs, i) => (
                    <Circle
                      key={`w-${i}`}
                      cx={toCanvasX(obs.x)}
                      cy={toCanvasY(obs.y)}
                      r={5.5}
                      fill={getObstacleColor(obs.kind)}
                      opacity={0.28}
                    />
                  ))}

                  {/* AI Learned Multi-Hazard Memory Perimeters */}
                  {(learned_zones || []).map((lz, idx) => (
                    <Circle
                      key={`lz-${idx}`}
                      cx={toCanvasX(lz.x)}
                      cy={toCanvasY(lz.y)}
                      r={10}
                      fill={getObstacleColor(lz.kind)}
                      opacity={0.22}
                      stroke={getObstacleColor(lz.kind)}
                      strokeWidth="1.5"
                    />
                  ))}

                  {/* Actively Scanned Costmap Cells */}
                  {(scanned_obstacles || []).map((cell, idx) => (
                    <Rect
                      key={`s-${idx}`}
                      x={toCanvasX(cell[0]) - 2.5}
                      y={toCanvasY(cell[1]) - 2.5}
                      width={5}
                      height={5}
                      fill="#1D1D1F"
                      opacity={0.75}
                      rx="1"
                    />
                  ))}

                  {/* Global A* Route Line (Apple System Blue) */}
                  {pathPointsString.length > 0 && (
                    <Polyline
                      points={pathPointsString}
                      fill="none"
                      stroke="#0071E3"
                      strokeWidth="3.5"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    />
                  )}

                  {/* Home Base Marker (Indigo Ring) */}
                  <Circle
                    cx={toCanvasX(home_cell[0])}
                    cy={toCanvasY(home_cell[1])}
                    r={7}
                    fill="#5856D6"
                    stroke="#FFFFFF"
                    strokeWidth="2"
                  />

                  {/* Goal Beacon (Apple Emerald Green) */}
                  <Circle cx={toCanvasX(goal_cell[0])} cy={toCanvasY(goal_cell[1])} r={10} fill="#34C759" opacity={0.25} />
                  <Circle cx={toCanvasX(goal_cell[0])} cy={toCanvasY(goal_cell[1])} r={5.5} fill="#34C759" stroke="#FFFFFF" strokeWidth="2" />

                  {/* Rover Position & Heading Indicator */}
                  <Line
                    x1={toCanvasX(rx)}
                    y1={toCanvasY(ry)}
                    x2={toCanvasX(rx + 4.8 * Math.cos(ryaw))}
                    y2={toCanvasY(ry + 4.8 * Math.sin(ryaw))}
                    stroke="#0071E3"
                    strokeWidth="2.5"
                  />
                  <Circle cx={toCanvasX(rx)} cy={toCanvasY(ry)} r={6.5} fill="#0071E3" stroke="#FFFFFF" strokeWidth="2.5" />
                </Svg>
              </div>
            </View>

            <View style={styles.legendStrip}>
              <Text style={styles.legendText}>🔵 Rover ({pose.x},{pose.y})</Text>
              <Text style={styles.legendText}>🟣 Home ({home_cell[0]},{home_cell[1]})</Text>
              <Text style={styles.legendText}>🟢 Goal ({goal_cell[0]},{goal_cell[1]})</Text>
              <Text style={styles.legendText}>🔴 Debris ({learned_counts.DEBRIS || 0})</Text>
              <Text style={styles.legendText}>🟢 Bio-Chem ({learned_counts.BIOCHEM || 0})</Text>
              <Text style={styles.legendText}>🟣 Rad ({learned_counts.RADIATION || 0})</Text>
            </View>
          </View>
        </View>

        {/* Home Base & Target Waypoint Control Strip */}
        <View style={styles.bentoCardFull}>
          <View style={styles.dualControlRow}>
            {/* Goal Controls */}
            <View style={styles.controlBlock}>
              <Text style={styles.cardEyebrow}>TARGET DESTINATION</Text>
              <View style={styles.inlineInputs}>
                <TextInput style={styles.appleInput} value={inputX} onChangeText={setInputX} placeholder="X" />
                <TextInput style={styles.appleInput} value={inputY} onChangeText={setInputY} placeholder="Y" />
                <TouchableOpacity style={styles.primaryAppleBtn} onPress={() => applyGoalCoordinates(inputX, inputY)}>
                  <Text style={styles.primaryAppleBtnText}>Route to Goal</Text>
                </TouchableOpacity>
                <TouchableOpacity style={styles.secondaryAppleBtn} onPress={() => applyGoalCoordinates(home_cell[0], home_cell[1])}>
                  <Text style={styles.secondaryAppleBtnText}>Return to Home</Text>
                </TouchableOpacity>
              </View>
            </View>

            {/* Home Base Configuration */}
            <View style={styles.controlBlock}>
              <Text style={styles.cardEyebrow}>HOME BASE STATION ({home_cell[0]}, {home_cell[1]})</Text>
              <View style={styles.inlineInputs}>
                <TextInput style={styles.appleInput} value={homeX} onChangeText={setHomeX} placeholder="HX" />
                <TextInput style={styles.appleInput} value={homeY} onChangeText={setHomeY} placeholder="HY" />
                <TouchableOpacity style={styles.secondaryAppleBtn} onPress={() => applyHomeCoordinates(homeX, homeY, false)}>
                  <Text style={styles.secondaryAppleBtnText}>Set Home</Text>
                </TouchableOpacity>
                <TouchableOpacity style={styles.secondaryAppleBtn} onPress={() => applyHomeCoordinates(0, 0, true)}>
                  <Text style={styles.secondaryAppleBtnText}>Use Current Pose</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={styles.indigoAppleBtn}
                  onPress={() => postAPI("/override", { autonomous_enabled: true, linear_v: 0, angular_w: 0, reset_sim: true })}
                >
                  <Text style={styles.primaryAppleBtnText}>Deploy from Home</Text>
                </TouchableOpacity>
              </View>
            </View>
          </View>

          {/* Quick Disaster Sector Presets */}
          <View style={styles.presetPillRow}>
            <TouchableOpacity style={styles.sectorPill} onPress={() => applyGoalCoordinates(35, 23)}>
              <Text style={styles.sectorPillText}>↗ Traverse Ramp Alpha (35, 23)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.sectorPill} onPress={() => applyGoalCoordinates(20, 31)}>
              <Text style={styles.sectorPillText}>↖ Traverse Ramp Bravo (20, 31)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.sectorPill} onPress={() => applyGoalCoordinates(26, 45)}>
              <Text style={styles.sectorPillText}>↑ Traverse Ramp Charlie (26, 45)</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.sectorPill} onPress={() => applyGoalCoordinates(30, 55)}>
              <Text style={styles.sectorPillText}>★ Forward Recon Zone (30, 55)</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* 4 Apple Minimalist KPI Metric Cards */}
        <View style={styles.kpiRow}>
          <View style={styles.kpiCard}>
            <Text style={styles.kpiLabel}>SPATIAL COORDINATES</Text>
            <Text style={styles.kpiValue}>
              {pose.x_float ?? pose.x}, {pose.y_float ?? pose.y}
            </Text>
            <Text style={styles.kpiSub}>Elevation {pose.z ?? 0.085}m</Text>
          </View>
          <View style={styles.kpiCard}>
            <Text style={styles.kpiLabel}>ATTITUDE & HEADING</Text>
            <Text style={styles.kpiValue}>{((pose.yaw || 0) * 57.3).toFixed(0)}° Yaw</Text>
            <Text style={styles.kpiSub}>Slope Pitch {((pose.pitch || 0) * 57.3).toFixed(1)}°</Text>
          </View>
          <View style={styles.kpiCard}>
            <Text style={styles.kpiLabel}>AI ADAPTATION INDEX</Text>
            <Text style={[styles.kpiValue, { color: "#0071E3" }]}>{safety_score}%</Text>
            <Text style={styles.kpiSub}>{learned_zones.length} Hazards Learned</Text>
          </View>
          <View style={styles.kpiCard}>
            <Text style={styles.kpiLabel}>ACTUATION TELEMETRY</Text>
            <Text style={styles.kpiValue}>{velocity.linear} m/s</Text>
            <Text style={styles.kpiSub}>PWM L:{Number(motors?.pwm_l || 0).toFixed(0)} R:{Number(motors?.pwm_r || 0).toFixed(0)}</Text>
          </View>
        </View>

        {/* Bottom Bento Row: Motor Actuation Curve + Neural Reconnaissance Feed */}
        <View style={styles.bentoGrid}>
          <View style={styles.bentoCard}>
            <Text style={styles.cardEyebrow}>CLOSED-LOOP C++17 PID CONTROLLER</Text>
            <Text style={styles.cardTitle}>4WD Motor PWM Actuation</Text>
            <View style={{ width: "100%", height: 145, marginTop: 10 }}>
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={history}>
                  <XAxis dataKey="time" stroke="#86868B" fontSize={10} />
                  <YAxis domain={[0, 255]} stroke="#86868B" fontSize={10} />
                  <Tooltip contentStyle={{ backgroundColor: "#FFFFFF", borderColor: "#E5E5EA", borderRadius: 10 }} />
                  <ReLine type="monotone" dataKey="pwmL" name="Left Bank" stroke="#0071E3" strokeWidth={2.2} dot={false} isAnimationActive={false} />
                  <ReLine type="monotone" dataKey="pwmR" name="Right Bank" stroke="#FF2D55" strokeWidth={2.2} dot={false} isAnimationActive={false} />
                </LineChart>
              </ResponsiveContainer>
            </View>
          </View>

          <View style={styles.bentoCard}>
            <Text style={styles.cardEyebrow}>AUTONOMOUS BLACK-BOX TELEMETRY</Text>
            <Text style={styles.cardTitle}>Lifelong Learning & Recon Log</Text>
            <View style={styles.cleanLogContainer}>
              {(mission_log || []).map((item, idx) => (
                <View key={idx} style={styles.logRow}>
                  <Text style={styles.cleanLogTime}>{item.time}</Text>
                  <Text style={styles.cleanLogEvent}>{item.event}</Text>
                </View>
              ))}
            </View>
          </View>
        </View>

        {/* Footer Hardware Override Dock */}
        <View style={styles.footerDock}>
          <TouchableOpacity
            style={[styles.dockBtn, { backgroundColor: autonomous ? "#0071E3" : "#86868B" }]}
            onPress={() => {
              const next = !autonomous;
              dispatch(setAutonomousMode(next));
              postAPI("/override", { autonomous_enabled: next, linear_v: 0, angular_w: 0 });
            }}
          >
            <Text style={styles.dockBtnText}>
              {autonomous ? "Autonomous Navigation Active (A* + APF)" : "Manual Standby Mode"}
            </Text>
          </TouchableOpacity>

          <TouchableOpacity
            style={[styles.dockBtn, { backgroundColor: "#FF3B30" }]}
            onPress={() => {
              dispatch(setAutonomousMode(false));
              postAPI("/override", { autonomous_enabled: false, linear_v: 0, angular_w: 0 });
            }}
          >
            <Text style={styles.dockBtnText}>Emergency Hardware Stop</Text>
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
  container: { flex: 1, backgroundColor: "#F5F5F7" },
  scroll: { padding: 20, maxWidth: 1260, alignSelf: "center", width: "100%" },
  navBar: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    backgroundColor: "#FFFFFF",
    paddingHorizontal: 20,
    paddingVertical: 14,
    borderRadius: 18,
    marginBottom: 16,
    borderWidth: 1,
    borderColor: "#E5E5EA",
    flexWrap: "wrap",
    gap: 12,
  },
  brandSubtitle: { color: "#86868B", fontSize: 10, fontWeight: "700", letterSpacing: 1.1 },
  brandTitle: { color: "#1D1D1F", fontSize: 20, fontWeight: "800", letterSpacing: -0.4 },
  segmentedControl: {
    flexDirection: "row",
    backgroundColor: "#F2F2F7",
    padding: 4,
    borderRadius: 12,
    flexWrap: "wrap",
  },
  segmentBtn: { paddingHorizontal: 12, paddingVertical: 7, borderRadius: 9 },
  segmentBtnActive: { backgroundColor: "#FFFFFF", shadowColor: "#000", shadowOpacity: 0.06, shadowRadius: 4 },
  segmentText: { color: "#6E6E73", fontSize: 12, fontWeight: "600" },
  segmentTextActive: { color: "#1D1D1F", fontWeight: "700" },
  statusRow: { flexDirection: "row", alignItems: "center" },
  pillBadge: { flexDirection: "row", alignItems: "center", paddingHorizontal: 12, paddingVertical: 6, borderRadius: 20 },
  dot: { width: 7, height: 7, borderRadius: 4, marginRight: 6 },
  pillText: { fontSize: 12, fontWeight: "700" },
  bentoGrid: { flexDirection: "row", flexWrap: "wrap", gap: 16, marginBottom: 16 },
  bentoCard: {
    flex: 1,
    minWidth: 340,
    backgroundColor: "#FFFFFF",
    padding: 18,
    borderRadius: 20,
    borderWidth: 1,
    borderColor: "#E5E5EA",
  },
  bentoCardFull: {
    backgroundColor: "#FFFFFF",
    padding: 18,
    borderRadius: 20,
    borderWidth: 1,
    borderColor: "#E5E5EA",
    marginBottom: 16,
  },
  cardHeader: { flexDirection: "row", justifyContent: "space-between", alignItems: "flex-start", marginBottom: 12 },
  cardEyebrow: { color: "#86868B", fontSize: 10, fontWeight: "700", letterSpacing: 0.9 },
  cardTitle: { color: "#1D1D1F", fontSize: 17, fontWeight: "700", marginTop: 2 },
  subtleChip: { backgroundColor: "#F2F2F7", paddingHorizontal: 10, paddingVertical: 5, borderRadius: 10 },
  subtleChipText: { color: "#0071E3", fontSize: 11, fontWeight: "700" },
  cameraContainer: { height: 265, backgroundColor: "#1D1D1F", borderRadius: 14, overflow: "hidden" },
  cameraImage: { width: "100%", height: "100%" },
  cameraEmpty: { flex: 1, justifyContent: "center", alignItems: "center" },
  emptyText: { color: "#86868B", fontSize: 13 },
  anomalyBar: { marginTop: 12 },
  microLabel: { color: "#86868B", fontSize: 10, fontWeight: "700", marginBottom: 6 },
  anomalyBtnGroup: { flexDirection: "row", gap: 8, flexWrap: "wrap" },
  anomalyChip: { flex: 1, minWidth: 110, paddingVertical: 7, paddingHorizontal: 8, borderRadius: 10, borderWidth: 1.2, alignItems: "center", backgroundColor: "#FAFAFC" },
  anomalyChipText: { fontSize: 11, fontWeight: "700" },
  mapModeToggle: { flexDirection: "row", backgroundColor: "#F2F2F7", padding: 3, borderRadius: 10 },
  modePill: { paddingHorizontal: 9, paddingVertical: 5, borderRadius: 8 },
  modePillActive: { backgroundColor: "#0071E3" },
  modePillActiveHome: { backgroundColor: "#5856D6" },
  modePillText: { color: "#6E6E73", fontSize: 11, fontWeight: "600" },
  modePillTextActive: { color: "#FFFFFF", fontWeight: "700" },
  mapCanvasContainer: { alignItems: "center", justifyContent: "center", backgroundColor: "#FAFAFC", padding: 8, borderRadius: 14, borderWidth: 1, borderColor: "#E5E5EA" },
  svgMap: { backgroundColor: "#FFFFFF", borderRadius: 10 },
  legendStrip: { flexDirection: "row", justifyContent: "space-around", marginTop: 10, flexWrap: "wrap", gap: 6 },
  legendText: { color: "#515154", fontSize: 11, fontWeight: "600" },
  dualControlRow: { flexDirection: "row", flexWrap: "wrap", gap: 20, justifyContent: "space-between" },
  controlBlock: { flex: 1, minWidth: 300 },
  inlineInputs: { flexDirection: "row", alignItems: "center", gap: 8, marginTop: 8, flexWrap: "wrap" },
  appleInput: {
    width: 52,
    backgroundColor: "#F2F2F7",
    color: "#1D1D1F",
    fontSize: 14,
    fontWeight: "700",
    textAlign: "center",
    paddingVertical: 8,
    borderRadius: 10,
  },
  primaryAppleBtn: { backgroundColor: "#0071E3", paddingHorizontal: 14, paddingVertical: 9, borderRadius: 10 },
  indigoAppleBtn: { backgroundColor: "#5856D6", paddingHorizontal: 14, paddingVertical: 9, borderRadius: 10 },
  primaryAppleBtnText: { color: "#FFFFFF", fontSize: 12, fontWeight: "700" },
  secondaryAppleBtn: { backgroundColor: "#F2F2F7", paddingHorizontal: 12, paddingVertical: 9, borderRadius: 10 },
  secondaryAppleBtnText: { color: "#1D1D1F", fontSize: 12, fontWeight: "600" },
  presetPillRow: { flexDirection: "row", gap: 8, marginTop: 14, flexWrap: "wrap" },
  sectorPill: { flex: 1, minWidth: 150, backgroundColor: "#F5F5F7", paddingVertical: 9, paddingHorizontal: 12, borderRadius: 10, alignItems: "center" },
  sectorPillText: { color: "#0071E3", fontSize: 12, fontWeight: "600" },
  kpiRow: { flexDirection: "row", gap: 14, marginBottom: 16, flexWrap: "wrap" },
  kpiCard: { flex: 1, minWidth: 180, backgroundColor: "#FFFFFF", padding: 16, borderRadius: 18, borderWidth: 1, borderColor: "#E5E5EA" },
  kpiLabel: { color: "#86868B", fontSize: 10, fontWeight: "700", letterSpacing: 0.8 },
  kpiValue: { color: "#1D1D1F", fontSize: 22, fontWeight: "800", marginTop: 4 },
  kpiSub: { color: "#6E6E73", fontSize: 12, fontWeight: "500", marginTop: 2 },
  cleanLogContainer: { height: 145, backgroundColor: "#F5F5F7", borderRadius: 12, padding: 10, marginTop: 10, overflow: "hidden" },
  logRow: { flexDirection: "row", marginBottom: 6 },
  cleanLogTime: { color: "#0071E3", fontSize: 11, fontWeight: "700", width: 62 },
  cleanLogEvent: { color: "#1D1D1F", fontSize: 11, fontWeight: "500", flex: 1 },
  footerDock: { flexDirection: "row", gap: 12, marginTop: 4 },
  dockBtn: { flex: 1, paddingVertical: 15, borderRadius: 14, alignItems: "center" },
  dockBtnText: { color: "#FFFFFF", fontSize: 13, fontWeight: "700" },
});
