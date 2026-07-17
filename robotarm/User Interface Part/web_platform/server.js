/*
  Restaurant Service Robot Backend
  --------------------------------
  Browser UI -> this server -> Favoriot -> ESP32 robot arm

  Why use this backend?
  - Favoriot API key stays in .env, not inside browser JavaScript.
  - The backend validates allowed robot commands before sending them to Favoriot.
  - Restaurant UI commands can be mapped to your existing ESP32 robot arm commands.
*/

import express from "express";
import path from "path";
import { fileURLToPath } from "url";
import { randomUUID } from "crypto";
import dotenv from "dotenv";

dotenv.config();

const app = express();

const PORT = process.env.PORT || 3000;

const FAVORIOT_BASE_URL =
  process.env.FAVORIOT_BASE_URL || "https://apiv2.favoriot.com/v2";

const FAVORIOT_STREAM_URL = `${FAVORIOT_BASE_URL}/streams`;

// Supports both naming styles in .env
const FAVORIOT_TOKEN =
  process.env.FAVORIOT_TOKEN ||
  process.env.FAVORIOT_API_KEY;

const FAVORIOT_DEVICE_DEVELOPER_ID =
  process.env.FAVORIOT_DEVICE_DEVELOPER_ID ||
  process.env.FAVORIOT_DEVICE_ID;

// For long eyJ... API key, use "apikey"
const FAVORIOT_TOKEN_HEADER =
  process.env.FAVORIOT_TOKEN_HEADER || "apikey";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// ============================================================
// COMMAND MAPPING
// ============================================================
// ui_cmd = command from HTML button
// robot_cmd = command sent to Favoriot and received by ESP32
//
// Your ESP32 Tag Recognition version currently supports:
// READY, BLUE, RED, YELLOW, AUTO_SORT, READ_TAG,
// AUTO_TAG_ON, AUTO_TAG_OFF, AUTO_TAG_TOGGLE,
// HOME, SAFE_UP, STOP, STATUS
//
// Old READ_COLOUR / READ_COLOR buttons are kept as aliases,
// but they now send READ_TAG to the ESP32.
const allowedCommands = new Map([
  ["READY", {
    robot_cmd: "READY",
    label: "Robot Ready"
  }],

  ["STATUS", {
    robot_cmd: "STATUS",
    label: "Check Robot Status"
  }],

  ["STOP", {
    robot_cmd: "STOP",
    label: "Emergency Stop"
  }],

  ["ESTOP", {
    robot_cmd: "STOP",
    label: "Emergency Stop"
  }],

  ["HOME", {
    robot_cmd: "HOME",
    label: "Return Home / Kitchen"
  }],

  ["RETURN", {
    robot_cmd: "HOME",
    label: "Return to Kitchen"
  }],

  ["RETURN_KITCHEN", {
    robot_cmd: "HOME",
    label: "Return to Kitchen"
  }],

  ["SAFE_UP", {
    robot_cmd: "SAFE_UP",
    label: "Move to Safe Up Position"
  }],

  ["AUTO_SORT", {
    robot_cmd: "AUTO_SORT",
    label: "Detect Tag Once and Sort"
  }],

  ["SCAN", {
    robot_cmd: "AUTO_SORT",
    label: "Detect Tag Once and Sort"
  }],

  ["READ_TAG", {
    robot_cmd: "READ_TAG",
    label: "Read Tag Once"
  }],

  // Backward compatible aliases for old UI buttons.
  // These no longer mean colour detection; they send READ_TAG to ESP32.
  ["READ_COLOUR", {
    robot_cmd: "READ_TAG",
    label: "Read Tag Once"
  }],

  ["READ_COLOR", {
    robot_cmd: "READ_TAG",
    label: "Read Tag Once"
  }],

  ["AUTO_TAG_ON", {
    robot_cmd: "AUTO_TAG_ON",
    label: "Turn Auto Tag Watch ON"
  }],

  ["AUTO_TAG_OFF", {
    robot_cmd: "AUTO_TAG_OFF",
    label: "Turn Auto Tag Watch OFF"
  }],

  ["AUTO_TAG_TOGGLE", {
    robot_cmd: "AUTO_TAG_TOGGLE",
    label: "Toggle Auto Tag Watch"
  }],

  // Optional: if your HTML button uses data-cmd="8"
  ["8", {
    robot_cmd: "AUTO_TAG_TOGGLE",
    label: "Toggle Auto Tag Watch"
  }],

  // Direct original robot arm sequence commands
  ["BLUE", {
    robot_cmd: "BLUE",
    label: "Run Blue Sequence"
  }],

  ["RED", {
    robot_cmd: "RED",
    label: "Run Red Sequence"
  }],

  ["YELLOW", {
    robot_cmd: "YELLOW",
    label: "Run Yellow Sequence"
  }],

  // Restaurant-themed table buttons
  // Temporarily mapped to your existing sequence commands
  ["TABLE_1", {
    robot_cmd: "BLUE",
    label: "Deliver to Table 1 / Blue Sequence"
  }],

  ["TABLE_2", {
    robot_cmd: "RED",
    label: "Deliver to Table 2 / Red Sequence"
  }],

  ["TABLE_3", {
    robot_cmd: "YELLOW",
    label: "Deliver to Table 3 / Yellow Sequence"
  }],

  ["TABLE_4", {
    robot_cmd: "AUTO_SORT",
    label: "Deliver to Table 4 / Auto Tag Sort"
  }],

  ["SERVICE", {
    robot_cmd: "AUTO_SORT",
    label: "Start Tag Service"
  }]
]);

let commandHistory = [];
let lastCommand = null;

// ============================================================
// MIDDLEWARE
// ============================================================
app.use(express.json());
app.use(express.static(path.join(__dirname, "public")));

// ============================================================
// HELPER FUNCTIONS
// ============================================================
function maskSecret(secret) {
  if (!secret || secret.length < 10) return "not configured";
  return `${secret.slice(0, 6)}...${secret.slice(-6)}`;
}

function normalizeCommand(cmd) {
  if (!cmd || typeof cmd !== "string") return "";
  return cmd.trim().toUpperCase();
}

function requireConfig() {
  if (!FAVORIOT_TOKEN) {
    throw new Error(
      "Missing Favoriot token. Please set FAVORIOT_TOKEN or FAVORIOT_API_KEY in .env"
    );
  }

  if (!FAVORIOT_DEVICE_DEVELOPER_ID) {
    throw new Error(
      "Missing Favoriot device ID. Please set FAVORIOT_DEVICE_DEVELOPER_ID or FAVORIOT_DEVICE_ID in .env"
    );
  }

  if (!FAVORIOT_DEVICE_DEVELOPER_ID.includes("@")) {
    throw new Error(
      "Favoriot device ID looks wrong. Example: robotArm@engloong5"
    );
  }
}

function parseJsonSafely(text) {
  try {
    return JSON.parse(text);
  } catch {
    return { raw: text };
  }
}

async function postCommandToFavoriot(uiCmd) {
  requireConfig();

  const cleanUiCmd = normalizeCommand(uiCmd);

  if (!allowedCommands.has(cleanUiCmd)) {
    const valid = Array.from(allowedCommands.keys()).join(", ");
    throw new Error(`Invalid command '${uiCmd}'. Valid commands: ${valid}`);
  }

  const commandInfo = allowedCommands.get(cleanUiCmd);
  const robotCmd = commandInfo.robot_cmd;
  const requestId = randomUUID();

  const body = {
    device_developer_id: FAVORIOT_DEVICE_DEVELOPER_ID,
    data: {
      cmd: robotCmd,
      ui_cmd: cleanUiCmd,
      request_id: requestId,
      label: commandInfo.label,
      source: "restaurant-service-dashboard",
      created_at: new Date().toISOString()
    }
  };

  console.log("Sending command to Favoriot:");
  console.log(JSON.stringify(body, null, 2));

  const response = await fetch(FAVORIOT_STREAM_URL, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "Accept": "application/json",
      "Cache-Control": "no-cache",

      // Example:
      // FAVORIOT_TOKEN_HEADER=apikey
      // This becomes:
      // apikey: FAVORIOT_TOKEN
      [FAVORIOT_TOKEN_HEADER]: FAVORIOT_TOKEN
    },
    body: JSON.stringify(body)
  });

  const text = await response.text();
  const favoriotResponse = parseJsonSafely(text);

  const record = {
    request_id: requestId,
    ui_cmd: cleanUiCmd,
    robot_cmd: robotCmd,
    label: commandInfo.label,
    sent_at: new Date().toISOString(),
    http_status: response.status,
    favoriot: favoriotResponse
  };

  lastCommand = record;
  commandHistory.unshift(record);
  commandHistory = commandHistory.slice(0, 20);

  if (!response.ok) {
    throw new Error(
      `Favoriot rejected command. HTTP ${response.status}: ${JSON.stringify(favoriotResponse)}`
    );
  }

  return record;
}

// ============================================================
// API ROUTES
// ============================================================
app.get("/api/health", (req, res) => {
  res.json({
    ok: true,
    server_time: new Date().toISOString(),
    favoriot_base_url: FAVORIOT_BASE_URL,
    favoriot_stream_url: FAVORIOT_STREAM_URL,
    favoriot_device_configured: Boolean(FAVORIOT_DEVICE_DEVELOPER_ID),
    favoriot_key_configured: Boolean(FAVORIOT_TOKEN),
    favoriot_device: FAVORIOT_DEVICE_DEVELOPER_ID || "missing",
    favoriot_token_header: FAVORIOT_TOKEN_HEADER,
    favoriot_token_preview: maskSecret(FAVORIOT_TOKEN),
    allowed_commands: Array.from(allowedCommands.keys())
  });
});

app.get("/api/history", (req, res) => {
  res.json({
    ok: true,
    lastCommand,
    history: commandHistory
  });
});

app.post("/api/command", async (req, res) => {
  try {
    const { cmd } = req.body || {};

    if (!cmd || typeof cmd !== "string") {
      return res.status(400).json({
        ok: false,
        error: "Body must include string field 'cmd'."
      });
    }

    const record = await postCommandToFavoriot(cmd);

    res.json({
      ok: true,
      message: `Command sent to Favoriot: ${record.robot_cmd}`,
      record
    });
  } catch (err) {
    console.error("Command error:", err.message);

    res.status(400).json({
      ok: false,
      error: err.message
    });
  }
});

// ============================================================
// SERVER START
// ============================================================
app.listen(PORT, () => {
  console.log("======================================");
  console.log(`Restaurant Robot Dashboard running at: http://localhost:${PORT}`);
  console.log("Favoriot stream URL:", FAVORIOT_STREAM_URL);
  console.log("Favoriot device:", FAVORIOT_DEVICE_DEVELOPER_ID || "missing");
  console.log("Favoriot token configured:", FAVORIOT_TOKEN ? "YES" : "NO");
  console.log("Favoriot token header:", FAVORIOT_TOKEN_HEADER);
  console.log("Favoriot token preview:", maskSecret(FAVORIOT_TOKEN));
  console.log("======================================");
});