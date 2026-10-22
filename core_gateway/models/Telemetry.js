// core_gateway/models/Telemetry.js
const mongoose = require("mongoose");

const TelemetrySchema = new mongoose.Schema({
  timestamp: { type: Number, required: true, index: true },
  pose: {
    x: Number,
    y: Number,
    yaw: Number,
  },
  vo_confidence: Number,
  velocity: {
    linear: Number,
    angular: Number,
  },
  motors: {
    pwm_l: Number,
    pwm_r: Number,
    dir_l: String,
    dir_r: String,
  },
  detections_count: Number,
});

module.exports = mongoose.model("Telemetry", TelemetrySchema);