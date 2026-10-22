const express = require("express");
const http = require("http");
const { Server } = require("socket.io");
const mongoose = require("mongoose");
const cors = require("cors");
const path = require("path");

const app = express();
app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: "*", methods: ["GET", "POST"] },
});

const MONGO_URI = process.env.MONGO_URI || "mongodb://127.0.0.1:27017/mech_minds_rover";
const EDGE_API_URL = process.env.EDGE_API_URL || "http://127.0.0.1:8000";

mongoose.connect(MONGO_URI).catch(() => {});

let frameCounter = 0;

io.on("connection", (socket) => {
  console.log(`[CoreGateway] Client connected: ${socket.id}`);

  socket.on("edge_telemetry", async (payload) => {
    io.emit("rover_telemetry_stream", payload);
  });

  socket.on("operator_command", async (cmd) => {
    try {
      const endpoint = cmd.type === "SET_GOAL" ? "/goal" : "/override";
      await fetch(`${EDGE_API_URL}${endpoint}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(cmd.payload),
      });
    } catch (err) {}
  });
});

const PORT = process.env.PORT || 4000;
server.listen(PORT, "0.0.0.0", () => {
  console.log(`[CoreGateway] Live at http://localhost:${PORT}`);
});
