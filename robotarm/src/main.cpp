/*
  FOREST CONTROL DECK + FAVORIOT + HUSKYLENS ROBOT ARM
  =====================================================
  Fast smooth blended-sequence version: adaptive timing + coordinated motion
  - Cleaner state transition logging
  - Stronger servo soft-limit handling
  - More stable HuskyLens confirmation
  - Better Favoriot config checking
  - Non-blocking WiFi reconnect helper
  - Emergency stop print-spam prevention
  - Synchronized quintic ease-in/ease-out servo motion

  What this final code does:
  1) Serial Monitor can still control/test the robot arm.
  2) Forest-theme web platform sends button commands to your Node.js server.
  3) Node.js server sends the command to Favoriot as a data stream.
  4) ESP32 polls Favoriot and runs the correct robot arm sequence.
  5) ESP32 also watches the MOBILE ROBOT device stream.
  6) When the mobile robot posts DONE / FINISH / *_complete, AUTO_SORT is queued.
  7) HuskyLens detects AprilTag ID1 / ID2 / ID3 and runs the matching smooth sequence.
  8) STOP from platform/Serial/physical button activates emergency lock.
  9) HOME is still allowed after emergency lock, unless physical E-stop is held.

  Hardware:
  - ESP32
  - PCA9685 servo driver on I2C: SDA=GPIO21, SCL=GPIO22, address 0x40
  - HuskyLens on UART2: HuskyLens TX/T -> ESP32 RX2 GPIO16, HuskyLens RX/R -> ESP32 TX2 GPIO17
  - Optional physical emergency stop: GPIO27 to GND, using INPUT_PULLUP

  Arduino libraries needed:
  - Adafruit PWM Servo Driver Library
  - HUSKYLENS library
  - ArduinoJson
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "HUSKYLENS.h"
#include <math.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
// ============================================================
// WIFI + FAVORIOT CONFIG
// ============================================================
// Change these before uploading.
// SECURITY: Do not share your real WiFi password or Favoriot API key in screenshots, GitHub, or chat.
// If you already exposed your API key, regenerate it in Favoriot and paste the new key here.

const char *WIFI_SSID = "...";
const char *WIFI_PASSWORD = "...";
const char *FAVORIOT_API_KEY = "...";
const char *FAVORIOT_COMMAND_DEVICE_ID = "...";

// The MOBILE ROBOT must post its completion status to this device.
// Change this string if your actual mobile-robot device developer ID is different.
// Accepted examples in data.status: DONE, FINISH, FINISHED, COMPLETE, COMPLETED,
// TaskMiddle_complete, TaskLeft_complete, TaskRight_complete.
const char *FAVORIOT_MOBILE_ROBOT_DEVICE_ID = "...";

const char *FAVORIOT_BASE_URL = "https://apiv2.favoriot.com/v2";
const char *FAVORIOT_RPC_URL = "https://apiv2.favoriot.com/v2/rpc";

// Some platforms accept robotArm@username directly in the path.
// If your server returns 404 for the device URL, change this to true.
const bool FAVORIOT_URL_ENCODE_DEVICE_ID = false;

// Keep true while debugging. It prints the Favoriot error body for HTTP 401/404/etc.
const bool FAVORIOT_PRINT_ERROR_BODY = true;

// Dashboard command stream and mobile completion stream are separate:
// - robotArm@engloong5 receives READY/AUTO_SORT/STOP/etc. from the web UI.
// - FAVORIOT_MOBILE_ROBOT_DEVICE_ID receives mobile robot status updates.

// Normal command polling. Lower = faster response but more API requests.
const unsigned long FAVORIOT_COMMAND_POLL_MS = 2500;

// Poll the mobile robot completion stream. 1200 ms is responsive without
// making an HTTPS request every 50 ms.
const unsigned long FAVORIOT_MOBILE_POLL_MS = 100;

// On ESP32 boot, remember the latest existing mobile stream but do not replay
// an old completion. Only a NEW completion posted after boot will trigger.
const bool IGNORE_OLD_MOBILE_COMPLETION_ON_BOOT = true;

// Emergency STOP polling during servo movement.
const unsigned long FAVORIOT_STOP_POLL_MS = 800;

// Non-blocking WiFi reconnect interval.
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;

unsigned long lastFavoriotCommandPoll = 0;
unsigned long lastFavoriotMobilePoll = 0;
unsigned long lastFavoriotStopPoll = 0;
unsigned long lastWiFiReconnectAttempt = 0;
String lastSeenFavoriotCommandKey = "";
String lastSeenMobileStreamKey = "";
bool mobileStreamBaselineInitialized = false;
bool pendingMobileAutoSort = false;
WiFiClientSecure favoriotClient;

// ============================================================
// ROBOT STATE MACHINE
// ============================================================
enum RobotState
{
  STATE_INIT,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ESTOP
};

RobotState robotState = STATE_INIT;
bool initPromptShown = false;

const char *robotStateName(RobotState state)
{
  switch (state)
  {
  case STATE_INIT:
    return "INIT";
  case STATE_IDLE:
    return "IDLE";
  case STATE_RUNNING:
    return "RUNNING";
  case STATE_ESTOP:
    return "ESTOP";
  default:
    return "UNKNOWN";
  }
}

void changeRobotState(RobotState newState)
{
  if (robotState == newState)
    return;

  Serial.print("\n[STATE] ");
  Serial.print(robotStateName(robotState));
  Serial.print(" -> ");
  Serial.println(robotStateName(newState));
  robotState = newState;
}

// ============================================================
// PCA9685 / SERVO SETTINGS
// ============================================================
#define PCA9685_ADDR 0x40
#define SERVOMIN 120
#define SERVOMAX 600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

const int CH_BASE = 0;
const int CH_SHOULDER = 1;
const int CH_ELBOW = 2;
const int CH_WRIST = 3;
const int CH_ROTATE = 4;
const int CH_GRIPPER = 5;

// ============================================================
// HUSKYLENS SETTINGS
// ============================================================
HUSKYLENS huskylens;
HardwareSerial huskySerial(2);

const int HUSKY_RX_PIN = 16;
const int HUSKY_TX_PIN = 17;
const long HUSKY_BAUD = 9600;

const int HUSKY_INIT_RETRIES = 5;
const int HUSKY_REQUEST_RETRIES = 5;
bool huskyLensReady = false;

// Train HuskyLens in Tag Recognition mode with Learn Multiple ON:
// Tag ID 1 = run BLUE sequence
// Tag ID 2 = run RED sequence
// Tag ID 3 = run YELLOW sequence
// Change these only if your HuskyLens learned different tag IDs.
const int TAG_ID_BLUE_SEQUENCE = 1;
const int TAG_ID_RED_SEQUENCE = 2;
const int TAG_ID_YELLOW_SEQUENCE = 3;

const int HUSKY_DETECT_TRIES = 8;
const int HUSKY_REQUIRED_SAME_READS = 2;
const int HUSKY_READ_DELAY_MS = 80;

// ============================================================
// HOME POSITION + CURRENT ANGLES
// ============================================================
const int HOME_BASE = 82;
const int HOME_SHOULDER = 118;
const int HOME_ELBOW = 64;
const int HOME_WRIST = 80;
const int HOME_ROTATE = 84;
const int HOME_GRIPPER = 0;

int baseAngle = HOME_BASE;
int shoulderAngle = HOME_SHOULDER;
int elbowAngle = HOME_ELBOW;
int wristAngle = HOME_WRIST;
int rotateAngle = HOME_ROTATE;
int gripperAngle = HOME_GRIPPER;

const int STEP_SIZE = 2;

// ============================================================
// SMOOTH MOTION SETTINGS
// ============================================================
// PCA9685 runs at 50 Hz, so one coordinated command frame every 20 ms
// matches the servo refresh period and avoids sending useless extra updates.
const int MOTION_FRAME_MS = 20;

// Standalone HOME/manual moves are intentionally a little gentler.
// Recorded sequences use a higher scale so each waypoint is not too slow.
const float NORMAL_MOTION_SPEED_SCALE = 0.95f;
const float SEQUENCE_MOTION_SPEED_SCALE = 1.20f;

// Nominal maximum joint speeds in degrees per second.
// Shoulder and elbow remain slightly slower because they carry more load.
const float BASE_SPEED_DPS = 75.0f;
const float SHOULDER_SPEED_DPS = 65.0f;
const float ELBOW_SPEED_DPS = 75.0f;
const float WRIST_SPEED_DPS = 95.0f;
const float ROTATE_SPEED_DPS = 110.0f;
const float GRIPPER_SPEED_DPS = 145.0f;

// Adaptive timing: large movements receive more time; small movements finish
// quickly instead of using the same fixed duration for every action step.
const int MIN_NORMAL_MOVE_DURATION_MS = 140;
const int MAX_NORMAL_MOVE_DURATION_MS = 4000;
const int MIN_SEQUENCE_SEGMENT_MS = 100;
const int MAX_SEQUENCE_SEGMENT_MS = 2600;

// Continuous sequence blending settings.
// 0.0 = stop at every waypoint, 1.0 = maximum calculated pass-through speed.
// 0.82 gives a smooth path without making corners too aggressive.
const float WAYPOINT_BLEND_FACTOR = 0.82f;
const int MAX_SEQUENCE_WAYPOINTS = 40;
const int GRIPPER_SETTLE_MS = 220;
const int FINAL_SEQUENCE_SETTLE_MS = 60;
const bool ISOLATE_GRIPPER_ACTIONS = true;

// Recommended tuning:
// - Still too slow: raise SEQUENCE_MOTION_SPEED_SCALE to 1.30.
// - Too fast/shaky: lower it to 1.00.
// - Corners still pause too much: raise WAYPOINT_BLEND_FACTOR to 0.90.
// - Corners swing too aggressively: lower it to 0.65.

// Kept for HOME/status compatibility and optional standalone action-step moves.
const float ACTION_PAUSE_SPEED_FACTOR = 0.25f;
const int MIN_ACTION_PAUSE_MS = 50;

bool stopFlag = false;
bool emergencyLocked = false;
bool robotBusy = false;

// Optional physical emergency stop button.
// Wire one side to GPIO27, the other side to GND.
// Pressed = LOW because INPUT_PULLUP is used.
const int ESTOP_BUTTON_PIN = 27;
const bool USE_PHYSICAL_ESTOP_BUTTON = true;

// ============================================================
// SERVO SOFT LIMITS
// ============================================================
const int SAFE_BASE_MIN = 30;
const int SAFE_BASE_MAX = 134;
const int SAFE_SHOULDER_MIN = 50;
const int SAFE_SHOULDER_MAX = 130;
const int SAFE_ELBOW_MIN = 60;
const int SAFE_ELBOW_MAX = 156;
const int SAFE_WRIST_MIN = 0;
const int SAFE_WRIST_MAX = 115;
const int SAFE_ROTATE_MIN = 0;
const int SAFE_ROTATE_MAX = 180;
const int SAFE_GRIPPER_MIN = 0;
const int SAFE_GRIPPER_MAX = 90;

const bool AUTO_CLAMP_RECORDED_ACTIONS_TO_SAFE_LIMITS = true;

struct Pose
{
  int base;
  int shoulder;
  int elbow;
  int wrist;
  int rotate;
  int gripper;
};

// Format: {Base, Shoulder, Elbow, Wrist, Rotate, Gripper, PauseMs}
struct ActionStep
{
  int base;
  int shoulder;
  int elbow;
  int wrist;
  int rotate;
  int gripper;
  int pauseMs;
};

// ============================================================
// PRE-RECORDED SEQUENCES
// ============================================================
ActionStep blueBlockToBlueBox[] = {
    {82, 118, 64, 80, 84, 0, 400},
    {82, 102, 64, 80, 84, 0, 400},
    {82, 102, 138, 80, 84, 0, 400},
    {80, 102, 138, 80, 84, 0, 400},
    {80, 102, 138, 64, 84, 0, 400},
    {80, 86, 126, 64, 84, 0, 400},
    {80, 86, 126, 64, 84, 68, 400},
    {80, 124, 126, 64, 84, 68, 400},
    {64, 124, 126, 64, 84, 68, 400},
    {64, 124, 102, 64, 84, 68, 400},
    {64, 80, 102, 64, 84, 68, 400},
    {64, 80, 102, 64, 84, 14, 400},
    {64, 114, 102, 64, 84, 14, 400},
    {82, 118, 64, 80, 84, 0, 400}};
const int blueBlockToBlueBoxCount = sizeof(blueBlockToBlueBox) / sizeof(blueBlockToBlueBox[0]);

ActionStep redBlockToRedBox[] = {
    {82, 118, 64, 80, 84, 0, 400},
    {82, 114, 64, 80, 84, 0, 400},
    {80, 76, 94, 34, 84, 0, 400},
    {80, 76, 94, 34, 84, 0, 400},
    {80, 76, 94, 34, 84, 0, 400},
    {80, 76, 94, 34, 84, 0, 400},
    {80, 76, 94, 34, 84, 62, 400},
    {80, 108, 94, 34, 84, 62, 400},
    {58, 110, 108, 30, 84, 62, 400},
    {58, 110, 108, 30, 84, 62, 400},
    {58, 110, 108, 30, 84, 62, 400},
    {58, 110, 108, 30, 84, 62, 400},
    {58, 110, 108, 30, 84, 18, 400},
    {80, 110, 108, 30, 84, 18, 400},
    {82, 118, 64, 80, 84, 0, 400}};
const int redBlockToRedBoxCount = sizeof(redBlockToRedBox) / sizeof(redBlockToRedBox[0]);

ActionStep yellowBlockToYellowBox[] = {
    {82, 118, 64, 80, 84, 0, 400},
    {80, 78, 100, 42, 84, 0, 400},
    {80, 78, 100, 42, 84, 0, 400},
    {80, 78, 100, 42, 84, 0, 400},
    {80, 78, 100, 42, 84, 0, 400},
    {80, 78, 100, 42, 84, 60, 400},
    {80, 122, 100, 42, 84, 60, 400},
    {48, 128, 116, 18, 84, 60, 400},
    {48, 128, 116, 18, 84, 60, 400},
    {48, 126, 114, 18, 84, 60, 400},
    {48, 128, 112, 14, 84, 60, 400},
    {48, 128, 112, 14, 84, 60, 400},
    {48, 128, 112, 14, 84, 34, 400},
    {78, 128, 112, 14, 84, 34, 400},
    {82, 118, 64, 80, 84, 0, 400}};
const int yellowBlockToYellowBoxCount = sizeof(yellowBlockToYellowBox) / sizeof(yellowBlockToYellowBox[0]);

// ============================================================
// BASIC HELPERS
// ============================================================
String urlEncode(const String &s)
{
  String encoded = "";
  const char *hex = "0123456789ABCDEF";
  for (unsigned int i = 0; i < s.length(); i++)
  {
    char c = s.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
    {
      encoded += c;
    }
    else
    {
      encoded += '%';
      encoded += hex[(c >> 4) & 0xF];
      encoded += hex[c & 0xF];
    }
  }
  return encoded;
}

int rawAngleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void getAngleLimitForChannel(int channel, int &minA, int &maxA)
{
  switch (channel)
  {
  case CH_BASE:
    minA = SAFE_BASE_MIN;
    maxA = SAFE_BASE_MAX;
    break;
  case CH_SHOULDER:
    minA = SAFE_SHOULDER_MIN;
    maxA = SAFE_SHOULDER_MAX;
    break;
  case CH_ELBOW:
    minA = SAFE_ELBOW_MIN;
    maxA = SAFE_ELBOW_MAX;
    break;
  case CH_WRIST:
    minA = SAFE_WRIST_MIN;
    maxA = SAFE_WRIST_MAX;
    break;
  case CH_ROTATE:
    minA = SAFE_ROTATE_MIN;
    maxA = SAFE_ROTATE_MAX;
    break;
  case CH_GRIPPER:
    minA = SAFE_GRIPPER_MIN;
    maxA = SAFE_GRIPPER_MAX;
    break;
  default:
    minA = 0;
    maxA = 180;
    break;
  }
}

int clampAngleForChannel(int channel, int angle)
{
  int minA, maxA;
  getAngleLimitForChannel(channel, minA, maxA);
  return constrain(angle, minA, maxA);
}

int safeAngleToPulse(int channel, int angle)
{
  int safeAngle = clampAngleForChannel(channel, angle);
  return rawAngleToPulse(safeAngle);
}

void writeServoPWM(int channel, int angle)
{
  int pulse = safeAngleToPulse(channel, angle);
  pwm.setPWM(channel, 0, pulse);
}

Pose currentPose()
{
  Pose p;
  p.base = baseAngle;
  p.shoulder = shoulderAngle;
  p.elbow = elbowAngle;
  p.wrist = wristAngle;
  p.rotate = rotateAngle;
  p.gripper = gripperAngle;
  return p;
}

bool withinServoLimits(Pose p)
{
  return (p.base >= SAFE_BASE_MIN && p.base <= SAFE_BASE_MAX &&
          p.shoulder >= SAFE_SHOULDER_MIN && p.shoulder <= SAFE_SHOULDER_MAX &&
          p.elbow >= SAFE_ELBOW_MIN && p.elbow <= SAFE_ELBOW_MAX &&
          p.wrist >= SAFE_WRIST_MIN && p.wrist <= SAFE_WRIST_MAX &&
          p.rotate >= SAFE_ROTATE_MIN && p.rotate <= SAFE_ROTATE_MAX &&
          p.gripper >= SAFE_GRIPPER_MIN && p.gripper <= SAFE_GRIPPER_MAX);
}

Pose actionStepToPose(ActionStep s)
{
  Pose p = {s.base, s.shoulder, s.elbow, s.wrist, s.rotate, s.gripper};

  if (AUTO_CLAMP_RECORDED_ACTIONS_TO_SAFE_LIMITS)
  {
    p.base = clampAngleForChannel(CH_BASE, p.base);
    p.shoulder = clampAngleForChannel(CH_SHOULDER, p.shoulder);
    p.elbow = clampAngleForChannel(CH_ELBOW, p.elbow);
    p.wrist = clampAngleForChannel(CH_WRIST, p.wrist);
    p.rotate = clampAngleForChannel(CH_ROTATE, p.rotate);
    p.gripper = clampAngleForChannel(CH_GRIPPER, p.gripper);
  }

  return p;
}

// ============================================================
// EMERGENCY STOP
// ============================================================
void emergencyStop(const char *source)
{
  stopFlag = true;

  // Avoid repeated ESTOP spam when the physical button is held down
  // or when the same STOP is checked many times during movement.
  if (emergencyLocked)
  {
    changeRobotState(STATE_ESTOP);
    return;
  }

  emergencyLocked = true;
  changeRobotState(STATE_ESTOP);

  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.print("[ESTOP] ACTIVATED BY: ");
  Serial.println(source);
  Serial.println("[ESTOP] Movement/sorting disabled.");
  Serial.println("[ESTOP] HOME is still allowed unless physical E-stop is held.");
  Serial.println("[ESTOP] Reset ESP32 to fully clear emergency lock.");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

bool checkHardwareEmergency()
{
  if (!USE_PHYSICAL_ESTOP_BUTTON)
    return false;

  if (digitalRead(ESTOP_BUTTON_PIN) == LOW)
  {
    emergencyStop("PHYSICAL BUTTON");
    return true;
  }
  return false;
}

// Forward declaration because movement checks cloud STOP.
bool checkFavoriotEmergencyStopOnly();

bool checkStopKey()
{
  if (checkHardwareEmergency())
    return true;

  if (checkFavoriotEmergencyStopOnly())
    return true;

  while (Serial.available())
  {
    char k = Serial.peek();
    if (k == 'K' || k == 'k')
    {
      Serial.read();
      emergencyStop("SERIAL K");
      return true;
    }

    // Do not consume normal command keys here.
    break;
  }
  return false;
}

bool delayWithStopCheck(int waitMs)
{
  if (emergencyLocked)
    return false;

  int elapsed = 0;
  while (elapsed < waitMs)
  {
    if (checkStopKey())
      return false;
    delay(20);
    elapsed += 20;
  }
  return true;
}

int fastActionPauseMs(int originalPauseMs)
{
  int scaled = (int)(originalPauseMs * ACTION_PAUSE_SPEED_FACTOR);
  if (scaled < MIN_ACTION_PAUSE_MS)
    scaled = MIN_ACTION_PAUSE_MS;
  return scaled;
}

// ============================================================
// MOVEMENT FUNCTIONS
// ============================================================
void updateCurrentAngle(int channel, int angle)
{
  if (channel == CH_BASE)
    baseAngle = angle;
  else if (channel == CH_SHOULDER)
    shoulderAngle = angle;
  else if (channel == CH_ELBOW)
    elbowAngle = angle;
  else if (channel == CH_WRIST)
    wristAngle = angle;
  else if (channel == CH_ROTATE)
    rotateAngle = angle;
  else if (channel == CH_GRIPPER)
    gripperAngle = angle;
}

void updateCurrentPose(Pose p)
{
  baseAngle = p.base;
  shoulderAngle = p.shoulder;
  elbowAngle = p.elbow;
  wristAngle = p.wrist;
  rotateAngle = p.rotate;
  gripperAngle = p.gripper;
}

int getCurrentAngleByChannel(int channel)
{
  if (channel == CH_BASE)
    return baseAngle;
  if (channel == CH_SHOULDER)
    return shoulderAngle;
  if (channel == CH_ELBOW)
    return elbowAngle;
  if (channel == CH_WRIST)
    return wristAngle;
  if (channel == CH_ROTATE)
    return rotateAngle;
  if (channel == CH_GRIPPER)
    return gripperAngle;
  return 90;
}

Pose poseWithChangedChannel(int channel, int newAngle)
{
  Pose p = currentPose();
  if (channel == CH_BASE)
    p.base = newAngle;
  else if (channel == CH_SHOULDER)
    p.shoulder = newAngle;
  else if (channel == CH_ELBOW)
    p.elbow = newAngle;
  else if (channel == CH_WRIST)
    p.wrist = newAngle;
  else if (channel == CH_ROTATE)
    p.rotate = newAngle;
  else if (channel == CH_GRIPPER)
    p.gripper = newAngle;
  return p;
}

Pose clampPoseToSafeLimits(Pose p)
{
  p.base = clampAngleForChannel(CH_BASE, p.base);
  p.shoulder = clampAngleForChannel(CH_SHOULDER, p.shoulder);
  p.elbow = clampAngleForChannel(CH_ELBOW, p.elbow);
  p.wrist = clampAngleForChannel(CH_WRIST, p.wrist);
  p.rotate = clampAngleForChannel(CH_ROTATE, p.rotate);
  p.gripper = clampAngleForChannel(CH_GRIPPER, p.gripper);
  return p;
}

bool isMovementAllowed(Pose nextP, bool printReason = false)
{
  if (!withinServoLimits(nextP))
  {
    if (printReason)
      Serial.println("Blocked: target pose is outside servo soft limits.");
    return false;
  }
  return true;
}

// Quintic smoother-step curve:
// velocity and acceleration are both zero at the beginning and end.
float smoothMotionCurve(float t)
{
  t = constrain(t, 0.0f, 1.0f);
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

int interpolateAngleSmooth(int startAngle, int targetAngle, float easedProgress)
{
  float value = startAngle + (targetAngle - startAngle) * easedProgress;
  return (int)lroundf(value);
}

float jointMoveTimeMs(int startAngle, int targetAngle,
                      float speedDps, float motionScale)
{
  float safeScale = max(motionScale, 0.10f);
  float safeSpeed = max(speedDps * safeScale, 1.0f);
  return (fabsf((float)targetAngle - (float)startAngle) / safeSpeed) * 1000.0f;
}

int calculateAdaptiveMoveDurationMs(Pose start, Pose target,
                                    float motionScale,
                                    int minimumDurationMs,
                                    int maximumDurationMs)
{
  float durationMs = 0.0f;

  durationMs = max(durationMs, jointMoveTimeMs(start.base, target.base,
                                               BASE_SPEED_DPS, motionScale));
  durationMs = max(durationMs, jointMoveTimeMs(start.shoulder, target.shoulder,
                                               SHOULDER_SPEED_DPS, motionScale));
  durationMs = max(durationMs, jointMoveTimeMs(start.elbow, target.elbow,
                                               ELBOW_SPEED_DPS, motionScale));
  durationMs = max(durationMs, jointMoveTimeMs(start.wrist, target.wrist,
                                               WRIST_SPEED_DPS, motionScale));
  durationMs = max(durationMs, jointMoveTimeMs(start.rotate, target.rotate,
                                               ROTATE_SPEED_DPS, motionScale));
  durationMs = max(durationMs, jointMoveTimeMs(start.gripper, target.gripper,
                                               GRIPPER_SPEED_DPS, motionScale));

  int result = (int)ceilf(durationMs);
  return constrain(result, minimumDurationMs, maximumDurationMs);
}

int calculateNormalMoveDurationMs(Pose start, Pose target)
{
  return calculateAdaptiveMoveDurationMs(
      start, target, NORMAL_MOTION_SPEED_SCALE,
      MIN_NORMAL_MOVE_DURATION_MS, MAX_NORMAL_MOVE_DURATION_MS);
}

int calculateSequenceSegmentDurationMs(Pose start, Pose target)
{
  return calculateAdaptiveMoveDurationMs(
      start, target, SEQUENCE_MOTION_SPEED_SCALE,
      MIN_SEQUENCE_SEGMENT_MS, MAX_SEQUENCE_SEGMENT_MS);
}

void writePoseChanges(Pose previous, Pose next)
{
  if (next.base != previous.base)
    writeServoPWM(CH_BASE, next.base);
  if (next.shoulder != previous.shoulder)
    writeServoPWM(CH_SHOULDER, next.shoulder);
  if (next.elbow != previous.elbow)
    writeServoPWM(CH_ELBOW, next.elbow);
  if (next.wrist != previous.wrist)
    writeServoPWM(CH_WRIST, next.wrist);
  if (next.rotate != previous.rotate)
    writeServoPWM(CH_ROTATE, next.rotate);
  if (next.gripper != previous.gripper)
    writeServoPWM(CH_GRIPPER, next.gripper);
}

// Core coordinated motion routine.
// Normal movement checks physical, Serial, and Favoriot emergency stop.
// Emergency HOME override checks only the physical E-stop and keeps the lock active.
bool smoothMoveToPoseInternal(Pose target, bool allowEmergencyHomeOverride)
{
  if (emergencyLocked && !allowEmergencyHomeOverride)
  {
    Serial.println("Pose movement rejected: emergency lock active.");
    return false;
  }

  target = clampPoseToSafeLimits(target);
  if (!isMovementAllowed(target, true))
    return false;

  Pose start = currentPose();
  if (start.base == target.base &&
      start.shoulder == target.shoulder &&
      start.elbow == target.elbow &&
      start.wrist == target.wrist &&
      start.rotate == target.rotate &&
      start.gripper == target.gripper)
  {
    return true;
  }

  int durationMs = calculateNormalMoveDurationMs(start, target);
  int totalFrames = max(1, (durationMs + MOTION_FRAME_MS - 1) / MOTION_FRAME_MS);
  Pose previous = start;

  for (int frame = 1; frame <= totalFrames; frame++)
  {
    if (allowEmergencyHomeOverride)
    {
      if (USE_PHYSICAL_ESTOP_BUTTON && digitalRead(ESTOP_BUTTON_PIN) == LOW)
      {
        Serial.println("HOME override stopped: physical E-stop pressed.");
        return false;
      }
    }
    else
    {
      if (checkStopKey())
        return false;
    }

    float progress = (float)frame / (float)totalFrames;
    float eased = smoothMotionCurve(progress);

    Pose next;
    next.base = interpolateAngleSmooth(start.base, target.base, eased);
    next.shoulder = interpolateAngleSmooth(start.shoulder, target.shoulder, eased);
    next.elbow = interpolateAngleSmooth(start.elbow, target.elbow, eased);
    next.wrist = interpolateAngleSmooth(start.wrist, target.wrist, eased);
    next.rotate = interpolateAngleSmooth(start.rotate, target.rotate, eased);
    next.gripper = interpolateAngleSmooth(start.gripper, target.gripper, eased);

    if (!isMovementAllowed(next, true))
      return false;

    // Every joint is updated in the same frame, so the arm follows one
    // coordinated path instead of moving one joint completely at a time.
    writePoseChanges(previous, next);
    updateCurrentPose(next);
    previous = next;

    delay(MOTION_FRAME_MS);
  }

  // Guarantee the exact final target is stored and commanded.
  writePoseChanges(previous, target);
  updateCurrentPose(target);
  return true;
}

bool safeMoveOneServo(int channel, int targetAngle)
{
  if (emergencyLocked)
  {
    Serial.println("Movement rejected: emergency lock active.");
    return false;
  }

  targetAngle = clampAngleForChannel(channel, targetAngle);
  Pose target = poseWithChangedChannel(channel, targetAngle);
  return smoothMoveToPoseInternal(target, false);
}

bool safeMoveToPose(Pose target)
{
  if (emergencyLocked)
  {
    Serial.println("Pose movement rejected: emergency lock active.");
    return false;
  }

  stopFlag = false;
  return smoothMoveToPoseInternal(target, false);
}

bool safeMoveToActionStep(ActionStep s)
{
  Pose target = actionStepToPose(s);

  if (!AUTO_CLAMP_RECORDED_ACTIONS_TO_SAFE_LIMITS && !withinServoLimits(target))
  {
    Serial.println("Action step blocked: outside servo soft limits.");
    return false;
  }

  if (!safeMoveToPose(target))
    return false;

  int pauseMs = fastActionPauseMs(s.pauseMs);
  if (!delayWithStopCheck(pauseMs))
    return false;

  return true;
}

bool posesAreEqual(Pose a, Pose b)
{
  return (a.base == b.base &&
          a.shoulder == b.shoulder &&
          a.elbow == b.elbow &&
          a.wrist == b.wrist &&
          a.rotate == b.rotate &&
          a.gripper == b.gripper);
}

bool armJointsChanged(Pose a, Pose b)
{
  return (a.base != b.base ||
          a.shoulder != b.shoulder ||
          a.elbow != b.elbow ||
          a.wrist != b.wrist ||
          a.rotate != b.rotate);
}

struct PoseVelocity
{
  float base;
  float shoulder;
  float elbow;
  float wrist;
  float rotate;
  float gripper;
};

PoseVelocity zeroPoseVelocity()
{
  PoseVelocity v = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  return v;
}

// Monotonic waypoint velocity calculation.
// A joint receives a non-zero through-velocity only when it continues moving
// in the same direction across the waypoint. Direction reversals stop smoothly.
float blendedWaypointVelocity(float previousAngle,
                              float currentAngle,
                              float nextAngle,
                              int previousDurationMs,
                              int nextDurationMs)
{
  if (previousDurationMs <= 0 || nextDurationMs <= 0)
    return 0.0f;

  float incomingSlope = (currentAngle - previousAngle) / (float)previousDurationMs;
  float outgoingSlope = (nextAngle - currentAngle) / (float)nextDurationMs;

  // Stop at a direction reversal or when either neighboring segment is still.
  if (incomingSlope == 0.0f || outgoingSlope == 0.0f ||
      incomingSlope * outgoingSlope <= 0.0f)
  {
    return 0.0f;
  }

  // Harmonic mean gives a monotonic tangent without overshoot.
  // The blend factor keeps enough through-speed for continuous motion while
  // reducing the chance of an aggressive corner on a loaded robot arm.
  float harmonicMean = (2.0f * incomingSlope * outgoingSlope) /
                       (incomingSlope + outgoingSlope);
  return harmonicMean * WAYPOINT_BLEND_FACTOR;
}

bool waypointTouchesGripperAction(Pose waypoints[], int waypointCount, int index)
{
  bool gripperChangedBefore =
      (index > 0 && waypoints[index - 1].gripper != waypoints[index].gripper);

  bool gripperChangedAfter =
      (index < waypointCount - 1 &&
       waypoints[index].gripper != waypoints[index + 1].gripper);

  return gripperChangedBefore || gripperChangedAfter;
}

PoseVelocity calculateWaypointVelocity(Pose waypoints[],
                                       int segmentDurationsMs[],
                                       int waypointCount,
                                       int index)
{
  PoseVelocity velocity = zeroPoseVelocity();

  // Start/end of the complete sequence must start and finish at zero speed.
  if (index <= 0 || index >= waypointCount - 1)
    return velocity;

  // Stop around gripper close/open operations so the object can be handled safely.
  if (waypointTouchesGripperAction(waypoints, waypointCount, index))
    return velocity;

  int previousDurationMs = segmentDurationsMs[index - 1];
  int nextDurationMs = segmentDurationsMs[index];

  Pose previous = waypoints[index - 1];
  Pose current = waypoints[index];
  Pose next = waypoints[index + 1];

  velocity.base = blendedWaypointVelocity(previous.base, current.base, next.base,
                                          previousDurationMs, nextDurationMs);
  velocity.shoulder = blendedWaypointVelocity(previous.shoulder, current.shoulder, next.shoulder,
                                              previousDurationMs, nextDurationMs);
  velocity.elbow = blendedWaypointVelocity(previous.elbow, current.elbow, next.elbow,
                                           previousDurationMs, nextDurationMs);
  velocity.wrist = blendedWaypointVelocity(previous.wrist, current.wrist, next.wrist,
                                           previousDurationMs, nextDurationMs);
  velocity.rotate = blendedWaypointVelocity(previous.rotate, current.rotate, next.rotate,
                                            previousDurationMs, nextDurationMs);
  velocity.gripper = blendedWaypointVelocity(previous.gripper, current.gripper, next.gripper,
                                             previousDurationMs, nextDurationMs);

  return velocity;
}

float cubicHermiteValue(float startValue,
                        float endValue,
                        float startVelocityPerMs,
                        float endVelocityPerMs,
                        int durationMs,
                        float t)
{
  t = constrain(t, 0.0f, 1.0f);
  float t2 = t * t;
  float t3 = t2 * t;

  float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
  float h10 = t3 - 2.0f * t2 + t;
  float h01 = -2.0f * t3 + 3.0f * t2;
  float h11 = t3 - t2;

  return h00 * startValue +
         h10 * startVelocityPerMs * durationMs +
         h01 * endValue +
         h11 * endVelocityPerMs * durationMs;
}

Pose interpolateBlendedPose(Pose start,
                            Pose target,
                            PoseVelocity startVelocity,
                            PoseVelocity endVelocity,
                            int durationMs,
                            float progress)
{
  Pose result;

  result.base = (int)lroundf(cubicHermiteValue(start.base, target.base,
                                               startVelocity.base, endVelocity.base,
                                               durationMs, progress));
  result.shoulder = (int)lroundf(cubicHermiteValue(start.shoulder, target.shoulder,
                                                   startVelocity.shoulder, endVelocity.shoulder,
                                                   durationMs, progress));
  result.elbow = (int)lroundf(cubicHermiteValue(start.elbow, target.elbow,
                                                startVelocity.elbow, endVelocity.elbow,
                                                durationMs, progress));
  result.wrist = (int)lroundf(cubicHermiteValue(start.wrist, target.wrist,
                                                startVelocity.wrist, endVelocity.wrist,
                                                durationMs, progress));
  result.rotate = (int)lroundf(cubicHermiteValue(start.rotate, target.rotate,
                                                 startVelocity.rotate, endVelocity.rotate,
                                                 durationMs, progress));
  result.gripper = (int)lroundf(cubicHermiteValue(start.gripper, target.gripper,
                                                  startVelocity.gripper, endVelocity.gripper,
                                                  durationMs, progress));

  return clampPoseToSafeLimits(result);
}

bool moveBlendedSequenceSegment(Pose start,
                                Pose target,
                                PoseVelocity startVelocity,
                                PoseVelocity endVelocity,
                                int durationMs)
{
  int totalFrames = max(1, (durationMs + MOTION_FRAME_MS - 1) / MOTION_FRAME_MS);
  Pose previousCommand = start;

  for (int frame = 1; frame <= totalFrames; frame++)
  {
    if (checkStopKey())
      return false;

    float progress = (float)frame / (float)totalFrames;
    Pose nextCommand = interpolateBlendedPose(start, target,
                                              startVelocity, endVelocity,
                                              durationMs, progress);

    if (!isMovementAllowed(nextCommand, true))
      return false;

    writePoseChanges(previousCommand, nextCommand);
    updateCurrentPose(nextCommand);
    previousCommand = nextCommand;

    delay(MOTION_FRAME_MS);
  }

  // Guarantee the exact final waypoint is commanded.
  writePoseChanges(previousCommand, target);
  updateCurrentPose(target);
  return true;
}

bool appendSequenceWaypoint(Pose waypoints[],
                            int waypointPausesMs[],
                            int &waypointCount,
                            Pose target,
                            int pauseMs)
{
  if (waypointCount <= 0)
    return false;

  target = clampPoseToSafeLimits(target);

  // Repeated recorded poses create unnecessary stop-start movement.
  // Merge them instead of replaying them.
  if (posesAreEqual(waypoints[waypointCount - 1], target))
  {
    waypointPausesMs[waypointCount - 1] =
        max(waypointPausesMs[waypointCount - 1], pauseMs);
    return true;
  }

  if (waypointCount >= MAX_SEQUENCE_WAYPOINTS)
  {
    Serial.println("Sequence rejected: too many expanded waypoints.");
    return false;
  }

  waypoints[waypointCount] = target;
  waypointPausesMs[waypointCount] = pauseMs;
  waypointCount++;
  return true;
}

bool playActionSafe(ActionStep action[], int stepCount, const char *actionName)
{
  if (emergencyLocked)
  {
    Serial.println("Action rejected: emergency lock active.");
    return false;
  }

  if (stepCount <= 0)
  {
    Serial.println("Action rejected: sequence has no steps.");
    return false;
  }

  robotBusy = true;
  changeRobotState(STATE_RUNNING);
  stopFlag = false;

  Serial.println();
  Serial.print("===== PLAYING CONTINUOUS SEQUENCE: ");
  Serial.print(actionName);
  Serial.println(" =====");
  Serial.println("Adaptive timing + waypoint blending enabled.");
  Serial.println("Small steps finish quickly; normal waypoints do not stop.");
  Serial.println("Only gripper actions and direction reversals slow to zero.");
  Serial.println("Serial K, platform STOP, or physical E-stop can stop movement.");

  Pose waypoints[MAX_SEQUENCE_WAYPOINTS];
  int waypointPausesMs[MAX_SEQUENCE_WAYPOINTS];
  int segmentDurationsMs[MAX_SEQUENCE_WAYPOINTS - 1];

  for (int i = 0; i < MAX_SEQUENCE_WAYPOINTS; i++)
    waypointPausesMs[i] = 0;

  int waypointCount = 1;
  waypoints[0] = currentPose();

  // Convert recorded ActionSteps into a clean continuous path.
  for (int i = 0; i < stepCount; i++)
  {
    Pose original = {action[i].base, action[i].shoulder, action[i].elbow,
                     action[i].wrist, action[i].rotate, action[i].gripper};
    Pose target = actionStepToPose(action[i]);

    if (!AUTO_CLAMP_RECORDED_ACTIONS_TO_SAFE_LIMITS && !withinServoLimits(target))
    {
      Serial.print("Action step outside servo limits: ");
      Serial.println(i + 1);
      robotBusy = false;
      changeRobotState(STATE_IDLE);
      return false;
    }

    if (AUTO_CLAMP_RECORDED_ACTIONS_TO_SAFE_LIMITS &&
        !posesAreEqual(original, target))
    {
      Serial.print("Step ");
      Serial.print(i + 1);
      Serial.println(" adjusted to servo soft limits.");
    }

    Pose previous = waypoints[waypointCount - 1];
    bool gripperChanged = previous.gripper != target.gripper;
    bool armChanged = armJointsChanged(previous, target);

    // Do not open/close the gripper while the arm is travelling.
    // Split a combined recorded step into arm movement first, then gripper action.
    if (ISOLATE_GRIPPER_ACTIONS && gripperChanged && armChanged)
    {
      Pose armOnlyTarget = target;
      armOnlyTarget.gripper = previous.gripper;

      if (!appendSequenceWaypoint(waypoints, waypointPausesMs,
                                  waypointCount, armOnlyTarget, 0))
      {
        robotBusy = false;
        changeRobotState(STATE_IDLE);
        return false;
      }
    }

    if (!appendSequenceWaypoint(waypoints, waypointPausesMs,
                                waypointCount, target, action[i].pauseMs))
    {
      robotBusy = false;
      changeRobotState(STATE_IDLE);
      return false;
    }
  }

  if (waypointCount <= 1)
  {
    Serial.println("Sequence already at its final pose.");
    robotBusy = false;
    changeRobotState(STATE_IDLE);
    return true;
  }

  for (int segment = 0; segment < waypointCount - 1; segment++)
  {
    segmentDurationsMs[segment] =
        calculateSequenceSegmentDurationMs(waypoints[segment], waypoints[segment + 1]);
  }

  Serial.print("Recorded steps: ");
  Serial.println(stepCount);
  Serial.print("Continuous waypoints after cleanup: ");
  Serial.println(waypointCount - 1);

  for (int segment = 0; segment < waypointCount - 1; segment++)
  {
    if (stopFlag || emergencyLocked)
    {
      Serial.println("Action stopped.");
      robotBusy = false;
      if (!emergencyLocked)
        changeRobotState(STATE_IDLE);
      return false;
    }

    PoseVelocity startVelocity =
        calculateWaypointVelocity(waypoints, segmentDurationsMs,
                                  waypointCount, segment);
    PoseVelocity endVelocity =
        calculateWaypointVelocity(waypoints, segmentDurationsMs,
                                  waypointCount, segment + 1);

    if (!moveBlendedSequenceSegment(waypoints[segment],
                                    waypoints[segment + 1],
                                    startVelocity, endVelocity,
                                    segmentDurationsMs[segment]))
    {
      Serial.println("Action stopped by safety/emergency condition.");
      robotBusy = false;
      if (!emergencyLocked)
        changeRobotState(STATE_IDLE);
      return false;
    }

    bool gripperMoved =
        waypoints[segment].gripper != waypoints[segment + 1].gripper;

    if (gripperMoved)
    {
      int recordedPause = fastActionPauseMs(waypointPausesMs[segment + 1]);
      int settleTime = max(GRIPPER_SETTLE_MS, recordedPause);

      if (!delayWithStopCheck(settleTime))
      {
        robotBusy = false;
        if (!emergencyLocked)
          changeRobotState(STATE_IDLE);
        return false;
      }
    }
  }

  if (!delayWithStopCheck(FINAL_SEQUENCE_SETTLE_MS))
  {
    robotBusy = false;
    if (!emergencyLocked)
      changeRobotState(STATE_IDLE);
    return false;
  }

  Serial.println("Continuous action finished smoothly.");
  robotBusy = false;
  changeRobotState(STATE_IDLE);
  return true;
}

void manualMove(const char *name, int channel, int delta)
{
  int current = getCurrentAngleByChannel(channel);
  int target = current + delta;

  Serial.print(name);
  Serial.print(" target: ");
  Serial.println(target);

  if (safeMoveOneServo(channel, target))
    Serial.println("Movement OK.");
  else
    Serial.println("Movement rejected/stopped.");
}

void moveAllCurrentServosUnsafeForStartupOnly()
{
  writeServoPWM(CH_BASE, baseAngle);
  writeServoPWM(CH_SHOULDER, shoulderAngle);
  writeServoPWM(CH_ELBOW, elbowAngle);
  writeServoPWM(CH_WRIST, wristAngle);
  writeServoPWM(CH_ROTATE, rotateAngle);
  writeServoPWM(CH_GRIPPER, gripperAngle);
}

// ============================================================
// HOME OVERRIDE AFTER EMERGENCY LOCK
// ============================================================
bool emergencyHomeMoveOneServo(int channel, int targetAngle)
{
  if (USE_PHYSICAL_ESTOP_BUTTON && digitalRead(ESTOP_BUTTON_PIN) == LOW)
  {
    Serial.println("HOME override rejected: physical E-stop is pressed.");
    return false;
  }

  targetAngle = clampAngleForChannel(channel, targetAngle);
  Pose target = poseWithChangedChannel(channel, targetAngle);
  return smoothMoveToPoseInternal(target, true);
}

bool goHomeAfterEmergencyLock()
{
  Serial.println();
  Serial.println("===== EMERGENCY HOME OVERRIDE =====");
  Serial.println("Emergency lock remains ON. Only HOME movement is allowed.");

  if (!emergencyHomeMoveOneServo(CH_GRIPPER, HOME_GRIPPER))
    return false;
  if (!emergencyHomeMoveOneServo(CH_WRIST, HOME_WRIST))
    return false;
  if (!emergencyHomeMoveOneServo(CH_SHOULDER, HOME_SHOULDER))
    return false;
  if (!emergencyHomeMoveOneServo(CH_ELBOW, HOME_ELBOW))
    return false;
  if (!emergencyHomeMoveOneServo(CH_BASE, HOME_BASE))
    return false;
  if (!emergencyHomeMoveOneServo(CH_ROTATE, HOME_ROTATE))
    return false;

  Serial.println("HOME reached while emergency lock is still active.");
  Serial.println("Reset ESP32 to clear emergency lock before sorting again.");
  return true;
}

void goHome()
{
  if (emergencyLocked)
  {
    goHomeAfterEmergencyLock();
    return;
  }

  Serial.println("Moving HOME...");
  Pose home = {HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, HOME_ROTATE, HOME_GRIPPER};
  if (safeMoveToPose(home))
    Serial.println("HOME reached.");
  else
    Serial.println("HOME movement stopped.");
}

void goSafeUp()
{
  if (emergencyLocked)
  {
    Serial.println("SAFE-UP rejected: emergency lock active. Use HOME only or reset ESP32.");
    return;
  }

  Serial.println("Moving SIMPLE SAFE-UP pose...");
  Pose p = currentPose();

  p.wrist = 90;
  if (!safeMoveToPose(p))
    return;

  p.shoulder = 125;
  if (!safeMoveToPose(p))
    return;

  p.elbow = 70;
  if (!safeMoveToPose(p))
    return;

  p.base = HOME_BASE;
  p.rotate = HOME_ROTATE;
  p.gripper = HOME_GRIPPER;
  if (!safeMoveToPose(p))
    return;

  Serial.println("SIMPLE SAFE-UP reached.");
}

// ============================================================
// HUSKYLENS TAG DETECTION
// ============================================================
const char *tagNameFromID(int id)
{
  if (id == TAG_ID_BLUE_SEQUENCE)
    return "BLUE";
  if (id == TAG_ID_RED_SEQUENCE)
    return "RED";
  if (id == TAG_ID_YELLOW_SEQUENCE)
    return "YELLOW";
  return "UNKNOWN";
}

bool isKnownTagID(int id)
{
  return (id == TAG_ID_BLUE_SEQUENCE || id == TAG_ID_RED_SEQUENCE || id == TAG_ID_YELLOW_SEQUENCE);
}

bool initHuskyLens()
{
  Serial.println();
  Serial.println("===== HUSKYLENS UART INIT =====");
  Serial.println("Expected wiring:");
  Serial.println("  HuskyLens TX/T -> ESP32 RX2 GPIO16");
  Serial.println("  HuskyLens RX/R -> ESP32 TX2 GPIO17");
  Serial.println("  VCC -> 5V, GND -> common GND");
  Serial.println("  HuskyLens Protocol Type -> Serial 9600 / UART");
  Serial.println("  Algorithm -> Tag Recognition");

  huskySerial.end();
  delay(100);
  huskySerial.begin(HUSKY_BAUD, SERIAL_8N1, HUSKY_RX_PIN, HUSKY_TX_PIN);
  delay(500);

  for (int attempt = 1; attempt <= HUSKY_INIT_RETRIES; attempt++)
  {
    Serial.print("HuskyLens init attempt ");
    Serial.print(attempt);
    Serial.print(" / ");
    Serial.println(HUSKY_INIT_RETRIES);

    if (huskylens.begin(huskySerial))
    {
      Serial.println("HuskyLens UART begin OK.");
      huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
      delay(500);
      huskyLensReady = true;
      Serial.println("HuskyLens ready in TAG RECOGNITION mode.");
      return true;
    }

    Serial.println("HuskyLens begin failed. Retrying...");
    delay(500);
  }

  Serial.println("HuskyLens UART init failed after retries.");
  huskyLensReady = false;
  return false;
}

int readDominantTagID()
{
  if (!huskyLensReady)
  {
    Serial.println("HuskyLens not ready. Reinitializing...");
    if (!initHuskyLens())
      return -1;
  }

  bool requestOK = false;

  for (int retry = 1; retry <= HUSKY_REQUEST_RETRIES; retry++)
  {
    if (huskylens.request())
    {
      requestOK = true;
      break;
    }

    Serial.print("HuskyLens request failed, retry ");
    Serial.print(retry);
    Serial.print(" / ");
    Serial.println(HUSKY_REQUEST_RETRIES);
    delay(150);

    if (retry == 3)
    {
      huskyLensReady = false;
      initHuskyLens();
    }
  }

  if (!requestOK)
  {
    Serial.println("HuskyLens request failed after retries.");
    huskyLensReady = false;
    return -1;
  }

  int bestID = 0;
  long bestArea = 0;

  while (huskylens.available())
  {
    HUSKYLENSResult result = huskylens.read();
    if (isKnownTagID(result.ID))
    {
      long area = (long)result.width * (long)result.height;
      if (area > bestArea)
      {
        bestArea = area;
        bestID = result.ID;
      }
    }
  }

  return bestID;
}

int readStableTagID()
{
  int lastID = 0;
  int sameCount = 0;

  for (int i = 0; i < HUSKY_DETECT_TRIES; i++)
  {
    if (checkStopKey())
      return 0;

    int id = readDominantTagID();

    if (id < 0)
    {
      delay(HUSKY_READ_DELAY_MS);
      continue;
    }

    if (id == 0)
    {
      sameCount = 0;
      lastID = 0;
      delay(HUSKY_READ_DELAY_MS);
      continue;
    }

    if (id == lastID)
      sameCount++;
    else
    {
      lastID = id;
      sameCount = 1;
    }

    Serial.print("HuskyLens sees: ");
    Serial.print(tagNameFromID(id));
    Serial.print(" ID=");
    Serial.print(id);
    Serial.print(" stability=");
    Serial.print(sameCount);
    Serial.print("/");
    Serial.println(HUSKY_REQUIRED_SAME_READS);

    if (sameCount >= HUSKY_REQUIRED_SAME_READS)
      return id;

    delay(HUSKY_READ_DELAY_MS);
  }

  return 0;
}

void readTagOnce()
{
  int id = readDominantTagID();
  if (id <= 0)
  {
    Serial.println("No trained TAG ID 1 / 2 / 3 block detected.");
    return;
  }

  Serial.print("Detected tag: ");
  Serial.print(tagNameFromID(id));
  Serial.print(" ID=");
  Serial.println(id);
}

bool sortDetectedTagOnce()
{
  if (emergencyLocked)
  {
    Serial.println("Tag sort rejected: emergency lock active.");
    return false;
  }

  Serial.println();
  Serial.println("===== TAG SORT ONE OBJECT =====");
  Serial.println("Reading HuskyLens tag ID...");

  int id = readStableTagID();

  if (id == 0)
  {
    Serial.println("No stable TAG ID 1 / 2 / 3 detected. Action cancelled.");
    return false;
  }

  Serial.print("Stable tag ID detected: ");
  Serial.println(tagNameFromID(id));

  if (id == TAG_ID_BLUE_SEQUENCE)
    return playActionSafe(blueBlockToBlueBox, blueBlockToBlueBoxCount, "BLUE block -> BLUE box");
  if (id == TAG_ID_RED_SEQUENCE)
    return playActionSafe(redBlockToRedBox, redBlockToRedBoxCount, "RED block -> RED box");
  if (id == TAG_ID_YELLOW_SEQUENCE)
    return playActionSafe(yellowBlockToYellowBox, yellowBlockToYellowBoxCount, "YELLOW block -> YELLOW box");

  return false;
}

// ============================================================
// I2C SCANNER
// ============================================================
bool checkI2CDevice(byte address)
{
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

int scanI2CBus()
{
  Serial.println();
  Serial.println("===== I2C BUS SCANNER =====");
  Serial.println("Expected: PCA9685 at 0x40. HuskyLens is UART, so it will not appear here.");

  int count = 0;
  bool foundPCA9685 = false;

  for (byte address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      count++;

      if (address == PCA9685_ADDR)
        foundPCA9685 = true;
    }
  }

  Serial.print("PCA9685 status: ");
  Serial.println(foundPCA9685 ? "FOUND" : "NOT FOUND");
  return count;
}

// ============================================================
// FAVORIOT FUNCTIONS
// ============================================================
bool ensureWiFiConnected(bool verbose = false)
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  if (millis() - lastWiFiReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS)
    return false;

  lastWiFiReconnectAttempt = millis();

  if (verbose)
    Serial.println("[WIFI] Disconnected. Trying non-blocking reconnect...");

  WiFi.disconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  return false;
}

void connectWiFiFavoriot()
{
  Serial.println();
  Serial.print("WIFI: connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - startAttempt > 15000)
    {
      Serial.println();
      Serial.println("WIFI: could not connect within 15s. Serial control still works.");
      return;
    }
  }

  Serial.println();
  Serial.print("WIFI: connected. IP = ");
  Serial.println(WiFi.localIP());
}

bool favoriotIsConfigured()
{
  String key = String(FAVORIOT_API_KEY);
  key.trim();

  if (key.length() < 15 ||
      key == "YOUR_FAVORIOT_API_KEY" ||
      key == "PASTE_YOUR_REAL_FAVORIOT_API_KEY_HERE")
  {
    Serial.println("FAVORIOT: API key is not configured. Paste your real API key first.");
    return false;
  }

  String device = String(FAVORIOT_COMMAND_DEVICE_ID);
  device.trim();
  if (device.length() == 0 ||
      device == "yourDevice@yourUsername" ||
      device.indexOf('@') < 0)
  {
    Serial.println("FAVORIOT: device developer ID looks wrong. Example: robotArm@username");
    return false;
  }

  return true;
}

String maskApiKeyForPrint()
{
  String key = String(FAVORIOT_API_KEY);
  key.trim();
  if (key.length() <= 8)
    return "not set";
  return key.substring(0, 4) + "..." + key.substring(key.length() - 4);
}

String favoriotPathID(const char *deviceID)
{
  String device = String(deviceID);
  device.trim();
  if (FAVORIOT_URL_ENCODE_DEVICE_ID)
    return urlEncode(device);
  return device;
}

String favoriotDevicePathID()
{
  return favoriotPathID(FAVORIOT_COMMAND_DEVICE_ID);
}

String favoriotMobileDevicePathID()
{
  return favoriotPathID(FAVORIOT_MOBILE_ROBOT_DEVICE_ID);
}

void addFavoriotHeaders(HTTPClient &http)
{
  // Favoriot v2 docs use the header name: apikey
  http.addHeader("apikey", FAVORIOT_API_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("Cache-Control", "no-cache");
}

void printFavoriotHttpError(int httpCode, const String &body)
{
  Serial.print("FAVORIOT: poll failed, HTTP code ");
  Serial.println(httpCode);

  if (httpCode == 401)
  {
    Serial.println("FAVORIOT: 401 means unauthorized. Most likely wrong API key, expired/regenerated key, or API key from a different Favoriot account.");
    Serial.println("FAVORIOT: Please paste the API key from the SAME account that owns this device.");
  }
  else if (httpCode == 404)
  {
    Serial.println("FAVORIOT: 404 means device/URL not found. Check FAVORIOT_COMMAND_DEVICE_ID exactly.");
    Serial.println("FAVORIOT: If your device ID contains @ and keeps failing, try FAVORIOT_URL_ENCODE_DEVICE_ID = true.");
  }
  else if (httpCode == 422)
  {
    Serial.println("FAVORIOT: 422 means the request format is not accepted.");
  }

  if (FAVORIOT_PRINT_ERROR_BODY && body.length() > 0)
  {
    Serial.println("FAVORIOT: response body below:");
    Serial.println(body.substring(0, 500));
  }
}

// Press J in Serial Monitor to call this.
// It checks whether the API key can access the configured device.
bool testFavoriotConnection()
{
  Serial.println();
  Serial.println("===== FAVORIOT CONNECTION TEST =====");

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("FAVORIOT TEST: WiFi is not connected.");
    ensureWiFiConnected(true);
    return false;
  }

  if (!favoriotIsConfigured())
    return false;

  favoriotClient.setInsecure(); // for student prototype/demo only

  HTTPClient http;
  String url = String(FAVORIOT_BASE_URL) + "/devices/" + favoriotDevicePathID();

  Serial.print("FAVORIOT TEST URL: ");
  Serial.println(url);
  Serial.print("FAVORIOT API KEY: ");
  Serial.println(maskApiKeyForPrint());

  http.begin(favoriotClient, url);
  http.setTimeout(8000);
  addFavoriotHeaders(http);

  int httpCode = http.GET();
  String body = http.getString();
  http.end();

  if (httpCode == 200)
  {
    Serial.println("FAVORIOT TEST: SUCCESS. API key and device ID are accepted.");
    Serial.println("====================================");
    return true;
  }

  Serial.println("FAVORIOT TEST: FAILED.");
  printFavoriotHttpError(httpCode, body);
  Serial.println("====================================");
  return false;
}

String getStreamKey(JsonObject item)
{
  const char *sid1 = item["stream_developer_id"] | "";
  const char *sid2 = item["stream_id"] | "";
  const char *sid3 = item["_id"] | "";

  if (strlen(sid1) > 0)
    return String(sid1);
  if (strlen(sid2) > 0)
    return String(sid2);
  if (strlen(sid3) > 0)
    return String(sid3);

  JsonObject data = item["data"].as<JsonObject>();
  const char *rid = data["request_id"] | "";
  if (strlen(rid) > 0)
    return String(rid);

  // Last fallback: use cmd/status + timestamp if provided by API.
  const char *cmd = data["cmd"] | "";
  const char *status = data["status"] | "";
  const char *created = item["created_at"] | "";
  if (strlen(cmd) > 0 || strlen(status) > 0 || strlen(created) > 0)
    return String(cmd) + "_" + String(status) + "_" + String(created);

  return "";
}

bool fetchLatestFavoriotCommand(String &cmdOut, String &keyOut)
{
  cmdOut = "";
  keyOut = "";

  if (WiFi.status() != WL_CONNECTED)
  {
    ensureWiFiConnected(false);
    return false;
  }

  if (!favoriotIsConfigured())
    return false;

  favoriotClient.setInsecure(); // for student prototype/demo only

  HTTPClient http;
  String url = String(FAVORIOT_BASE_URL) + "/devices/" + favoriotDevicePathID() + "/streams?max=5&order=desc";

  http.begin(favoriotClient, url);
  http.setTimeout(8000);
  addFavoriotHeaders(http);

  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  if (httpCode != 200)
  {
    printFavoriotHttpError(httpCode, payload);
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    Serial.print("FAVORIOT: JSON parse failed - ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.isNull() || results.size() == 0)
  {
    // No streams yet. This is not an error.
    return false;
  }

  // Pick newest stream that contains data.cmd.
  for (JsonObject item : results)
  {
    JsonObject data = item["data"].as<JsonObject>();
    if (data.isNull())
      continue;

    const char *cmdChar = data["cmd"] | "";
    if (strlen(cmdChar) == 0)
      continue;

    String streamKey = getStreamKey(item);

    if (streamKey.length() > 0 && streamKey == lastSeenFavoriotCommandKey)
      return false;

    cmdOut = String(cmdChar);
    cmdOut.trim();
    cmdOut.toUpperCase();
    keyOut = streamKey;
    return true;
  }

  return false;
}

bool checkFavoriotEmergencyStopOnly()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    ensureWiFiConnected(false);
    return false;
  }
  if (millis() - lastFavoriotStopPoll < FAVORIOT_STOP_POLL_MS)
    return false;
  lastFavoriotStopPoll = millis();

  String cmd, key;
  if (!fetchLatestFavoriotCommand(cmd, key))
    return false;

  if (cmd == "STOP" || cmd == "ESTOP")
  {
    if (key.length() > 0)
      lastSeenFavoriotCommandKey = key;
    emergencyStop("FAVORIOT PLATFORM");
    return true;
  }

  // Do not mark non-STOP command as seen here. It can be handled after movement finishes.
  return false;
}

// Forward declaration: the automatic handoff routes through the same command
// handler used by the dashboard and Serial Monitor.
void executeCommand(String cmd, const char *source);

// ============================================================
// MOBILE ROBOT -> ROBOT ARM AUTOMATIC HANDOFF
// ============================================================
// Expected mobile-robot stream example:
// {
//   "device_developer_id": "Ultrasonic_1@engloong5",
//   "data": { "status": "TaskMiddle_complete" }
// }
//
// The parser also accepts data.cmd so the mobile robot may send
// {"data":{"cmd":"DONE"}} if that is easier in its code.

bool mobileRobotDeviceIsConfigured()
{
  String device = String(FAVORIOT_MOBILE_ROBOT_DEVICE_ID);
  device.trim();

  if (device.length() == 0 || device.indexOf('@') < 0)
  {
    Serial.println("MOBILE TRIGGER: invalid mobile robot device developer ID.");
    return false;
  }

  return true;
}

bool isMobileCompletionSignal(String status)
{
  status.trim();
  status.toUpperCase();
  status.replace(" ", "_");
  status.replace("-", "_");

  // Avoid treating an explicit negative status as complete.
  if (status.startsWith("NOT_") || status.startsWith("NO_"))
    return false;

  if (status == "DONE" ||
      status == "FINISH" ||
      status == "FINISHED" ||
      status == "COMPLETE" ||
      status == "COMPLETED")
  {
    return true;
  }

  return status.endsWith("_DONE") ||
         status.endsWith("_FINISH") ||
         status.endsWith("_FINISHED") ||
         status.endsWith("_COMPLETE") ||
         status.endsWith("_COMPLETED");
}

bool fetchLatestMobileRobotStatus(String &statusOut, String &keyOut)
{
  statusOut = "";
  keyOut = "";

  if (WiFi.status() != WL_CONNECTED)
  {
    ensureWiFiConnected(false);
    return false;
  }

  if (!favoriotIsConfigured() || !mobileRobotDeviceIsConfigured())
    return false;

  favoriotClient.setInsecure(); // student prototype/demo only

  HTTPClient http;
  String url = String(FAVORIOT_BASE_URL) + "/devices/" +
               favoriotMobileDevicePathID() +
               "/streams?max=5&order=desc";

  http.begin(favoriotClient, url);
  http.setTimeout(8000);
  addFavoriotHeaders(http);

  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  if (httpCode != 200)
  {
    Serial.print("MOBILE TRIGGER: Favoriot poll failed. HTTP ");
    Serial.println(httpCode);
    if (FAVORIOT_PRINT_ERROR_BODY && payload.length() > 0)
      Serial.println(payload.substring(0, 300));
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    Serial.print("MOBILE TRIGGER: JSON parse failed - ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.isNull() || results.size() == 0)
    return false;

  // Results are requested newest first. Pick the newest stream containing
  // data.status. data.cmd is accepted as a fallback.
  for (JsonObject item : results)
  {
    JsonObject data = item["data"].as<JsonObject>();
    if (data.isNull())
      continue;

    const char *statusChar = data["status"] | "";
    const char *cmdChar = data["cmd"] | "";

    String status = strlen(statusChar) > 0 ? String(statusChar) : String(cmdChar);

    // Also accept {"done":true}, {"finished":true}, or {"complete":true}.
    if (status.length() == 0 && (data["done"] | false))
      status = "DONE";
    if (status.length() == 0 && (data["finished"] | false))
      status = "FINISHED";
    if (status.length() == 0 && (data["complete"] | false))
      status = "COMPLETE";

    status.trim();
    if (status.length() == 0)
      continue;

    statusOut = status;
    keyOut = getStreamKey(item);
    return true;
  }

  return false;
}

void checkMobileRobotCompletionSignal()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    ensureWiFiConnected(false);
    return;
  }

  // Do not perform an HTTPS request while servos are executing a sequence.
  // The latest completion remains in Favoriot and is read after the arm is idle.
  if (robotBusy || robotState == STATE_RUNNING)
    return;

  if (millis() - lastFavoriotMobilePoll < FAVORIOT_MOBILE_POLL_MS)
    return;
  lastFavoriotMobilePoll = millis();

  String status;
  String key;
  if (!fetchLatestMobileRobotStatus(status, key))
    return;

  // Establish a startup baseline so an old *_complete message does not make
  // the arm move immediately after reset.
  if (!mobileStreamBaselineInitialized)
  {
    mobileStreamBaselineInitialized = true;

    Serial.print("MOBILE TRIGGER: startup baseline = ");
    Serial.println(status);

    if (IGNORE_OLD_MOBILE_COMPLETION_ON_BOOT)
    {
      lastSeenMobileStreamKey = key;
      return;
    }
  }

  if (key.length() > 0 && key == lastSeenMobileStreamKey)
    return;

  // Mark every new stream as seen, including non-completion status updates.
  if (key.length() > 0)
    lastSeenMobileStreamKey = key;

  Serial.print("MOBILE TRIGGER: new mobile status -> ");
  Serial.println(status);

  if (!isMobileCompletionSignal(status))
  {
    Serial.println("MOBILE TRIGGER: status is not a completion signal.");
    return;
  }

  if (emergencyLocked || robotState == STATE_ESTOP)
  {
    Serial.println("MOBILE TRIGGER: completion ignored because ESTOP is active.");
    return;
  }

  // Queue instead of directly running here. The main loop starts AUTO_SORT
  // only when the arm is READY and IDLE.
  pendingMobileAutoSort = true;
  Serial.println("MOBILE TRIGGER: completion accepted; AUTO_SORT queued.");
}

void runPendingMobileAutoSortIfReady()
{
  static bool waitingForReadyMessageShown = false;

  if (!pendingMobileAutoSort)
  {
    waitingForReadyMessageShown = false;
    return;
  }

  if (emergencyLocked || robotState == STATE_ESTOP)
    return;

  if (robotState == STATE_INIT)
  {
    if (!waitingForReadyMessageShown)
    {
      Serial.println("MOBILE TRIGGER: AUTO_SORT waiting for READY.");
      waitingForReadyMessageShown = true;
    }
    return;
  }

  if (robotBusy || robotState != STATE_IDLE)
    return;

  waitingForReadyMessageShown = false;
  pendingMobileAutoSort = false;
  Serial.println();
  Serial.println("=================================================");
  Serial.println(" MOBILE ROBOT DONE -> STARTING ARM AUTO_SORT");
  Serial.println("=================================================");
  executeCommand("AUTO_SORT", "MOBILE ROBOT COMPLETE");
}

void printMenu();
void printServoStatus();
void printSafetyStatus();
void executeCommand(String cmd, const char *source);

void checkFavoriotDashboardCommand()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    ensureWiFiConnected(false);
    return;
  }
  if (robotBusy)
    return;
  if (millis() - lastFavoriotCommandPoll < FAVORIOT_COMMAND_POLL_MS)
    return;
  lastFavoriotCommandPoll = millis();

  String cmd, key;
  if (!fetchLatestFavoriotCommand(cmd, key))
    return;

  if (key.length() > 0)
    lastSeenFavoriotCommandKey = key;

  Serial.print("FAVORIOT: platform command received -> ");
  Serial.println(cmd);
  executeCommand(cmd, "FAVORIOT");
}

// ============================================================
// STATUS / MENU
// ============================================================
void printServoStatus()
{
  Serial.println();
  Serial.println("===== SERVO STATUS =====");
  Serial.print("Base     : ");
  Serial.println(baseAngle);
  Serial.print("Shoulder : ");
  Serial.println(shoulderAngle);
  Serial.print("Elbow    : ");
  Serial.println(elbowAngle);
  Serial.print("Wrist    : ");
  Serial.println(wristAngle);
  Serial.print("Rotate   : ");
  Serial.println(rotateAngle);
  Serial.print("Gripper  : ");
  Serial.println(gripperAngle);
  Serial.print("Robot state: ");
  Serial.println(robotStateName(robotState));
  Serial.print("Emergency lock: ");
  Serial.println(emergencyLocked ? "ON" : "OFF");
  Serial.println("========================");
}

void printSafetyStatus()
{
  Pose p = currentPose();
  Serial.println();
  Serial.println("===== ROBOT STATUS =====");
  Serial.println("Mode: Smooth synchronized ease-in/ease-out + servo soft limits + emergency stop + platform control");
  Serial.print("Servo-limit pose safe: ");
  Serial.println(withinServoLimits(p) ? "YES" : "NO");
  Serial.print("WiFi: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "not connected");
  Serial.print("Favoriot command device: ");
  Serial.println(FAVORIOT_COMMAND_DEVICE_ID);
  Serial.print("Mobile robot trigger device: ");
  Serial.println(FAVORIOT_MOBILE_ROBOT_DEVICE_ID);
  Serial.print("Mobile completion pending: ");
  Serial.println(pendingMobileAutoSort ? "YES" : "NO");
  Serial.print("Last mobile stream key: ");
  Serial.println(lastSeenMobileStreamKey.length() > 0 ? lastSeenMobileStreamKey : "none yet");
  Serial.print("Favoriot API key: ");
  Serial.println(maskApiKeyForPrint());
  Serial.print("Favoriot URL encode device ID: ");
  Serial.println(FAVORIOT_URL_ENCODE_DEVICE_ID ? "YES" : "NO");
  Serial.print("Last Favoriot command key: ");
  Serial.println(lastSeenFavoriotCommandKey.length() > 0 ? lastSeenFavoriotCommandKey : "none yet");
  Serial.print("HuskyLens ready: ");
  Serial.println(huskyLensReady ? "YES" : "NO");
  printServoStatus();
}

void printMenu()
{
  Serial.println();
  Serial.println("=================================================");
  Serial.println(" FOREST CONTROL DECK ROBOT ARM - SMOOTH TAG SORTING");
  Serial.println("=================================================");
  Serial.println("Initial ready:");
  Serial.println("Y = confirm robot READY and go HOME");
  Serial.println();
  Serial.println("Main tag sorting:");
  Serial.println("L = read HuskyLens tag ID once");
  Serial.println("T = detect ONE tag and run matching sequence");
  Serial.println("3 = run BLUE sequence");
  Serial.println("4 = run RED sequence");
  Serial.println("5 = run YELLOW sequence");
  Serial.println();
  Serial.println("Manual movement:");
  Serial.println("W/S = Shoulder up/down");
  Serial.println("A/D = Base left/right");
  Serial.println("Q/E = Elbow down/up");
  Serial.println("R/F = Wrist up/down");
  Serial.println("Z/X = Rotate left/right");
  Serial.println("C/V = Gripper close/open");
  Serial.println();
  Serial.println("Safety/status:");
  Serial.println("H = HOME");
  Serial.println("G = simple SAFE-UP");
  Serial.println("P = print servo angles");
  Serial.println("U = status");
  Serial.println("I = scan I2C + reinit HuskyLens");
  Serial.println("J = test Favoriot API/device connection");
  Serial.println("K = emergency stop lock");
  Serial.println("M = menu");
  Serial.println();
  Serial.println("Platform commands accepted through Favoriot:");
  Serial.println("READY, BLUE, RED, YELLOW, AUTO_SORT, READ_TAG, READ_COLOUR, HOME, SAFE_UP, STOP, STATUS");
  Serial.println();
  Serial.println("Automatic handoff:");
  Serial.println("Mobile robot data.status DONE/FINISH/*_complete -> queue AUTO_SORT");
  Serial.println("AUTO_SORT -> read AprilTag -> run matching smooth colour sequence");
  Serial.println("=================================================");
}

// ============================================================
// COMMAND ROUTER
// ============================================================
void confirmRobotReady()
{
  if (emergencyLocked)
  {
    Serial.println("READY rejected: emergency lock active. Reset ESP32 to clear.");
    return;
  }

  Serial.println("Robot READY confirmed.");
  goHome();
  changeRobotState(STATE_IDLE);
  initPromptShown = true;
  Serial.println("IDLE READY. Platform/Serial commands are now accepted.");
}

void executeCommand(String cmd, const char *source)
{
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "STOP" || cmd == "ESTOP")
  {
    emergencyStop(source);
    return;
  }

  if (cmd == "STATUS")
  {
    printSafetyStatus();
    return;
  }

  if (cmd == "MENU")
  {
    printMenu();
    return;
  }

  if (cmd == "HOME")
  {
    goHome();
    return;
  }

  if (emergencyLocked)
  {
    Serial.println("Command rejected: emergency lock active. HOME only, or reset ESP32.");
    return;
  }

  if (cmd == "READY" || cmd == "Y" || cmd == "YES")
  {
    confirmRobotReady();
    return;
  }

  if (robotState == STATE_INIT)
  {
    Serial.println("Robot is still in INIT. Send READY first, or press Y in Serial Monitor.");
    return;
  }

  if (robotState == STATE_RUNNING || robotBusy)
  {
    Serial.println("Robot is busy. Command ignored except STOP.");
    return;
  }

  if (cmd == "BLUE")
  {
    playActionSafe(blueBlockToBlueBox, blueBlockToBlueBoxCount, "BLUE block -> BLUE box");
  }
  else if (cmd == "RED")
  {
    playActionSafe(redBlockToRedBox, redBlockToRedBoxCount, "RED block -> RED box");
  }
  else if (cmd == "YELLOW")
  {
    playActionSafe(yellowBlockToYellowBox, yellowBlockToYellowBoxCount, "YELLOW block -> YELLOW box");
  }
  else if (cmd == "AUTO_SORT" || cmd == "SCAN" || cmd == "T")
  {
    sortDetectedTagOnce();
  }
  else if (cmd == "READ_TAG" || cmd == "READ_COLOUR" || cmd == "READ_COLOR" || cmd == "READ_TAG_ID" || cmd == "L")
  {
    readTagOnce();
  }
  else if (cmd == "SAFE_UP")
  {
    goSafeUp();
  }
  else
  {
    Serial.print("Unknown command from ");
    Serial.print(source);
    Serial.print(": ");
    Serial.println(cmd);
  }
}

void handleSerialKey(char key)
{
  if (key >= 'a' && key <= 'z')
    key -= 32;

  if (key == '\n' || key == '\r')
    return;

  switch (key)
  {
  case 'Y':
    executeCommand("READY", "SERIAL");
    break;
  case 'L':
    executeCommand("READ_TAG", "SERIAL");
    break;
  case 'T':
    executeCommand("AUTO_SORT", "SERIAL");
    break;
  case '3':
    executeCommand("BLUE", "SERIAL");
    break;
  case '4':
    executeCommand("RED", "SERIAL");
    break;
  case '5':
    executeCommand("YELLOW", "SERIAL");
    break;
  case 'H':
    executeCommand("HOME", "SERIAL");
    break;
  case 'G':
    executeCommand("SAFE_UP", "SERIAL");
    break;
  case 'U':
    executeCommand("STATUS", "SERIAL");
    break;
  case 'P':
    printServoStatus();
    break;
  case 'M':
    printMenu();
    break;
  case 'I':
    scanI2CBus();
    initHuskyLens();
    break;
  case 'J':
    testFavoriotConnection();
    break;
  case 'K':
    executeCommand("STOP", "SERIAL");
    break;

  // Manual movement commands are only allowed in IDLE and no emergency.
  case 'W':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Shoulder", CH_SHOULDER, STEP_SIZE);
    break;
  case 'S':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Shoulder", CH_SHOULDER, -STEP_SIZE);
    break;
  case 'A':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Base", CH_BASE, STEP_SIZE);
    break;
  case 'D':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Base", CH_BASE, -STEP_SIZE);
    break;
  case 'Q':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Elbow", CH_ELBOW, -STEP_SIZE);
    break;
  case 'E':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Elbow", CH_ELBOW, STEP_SIZE);
    break;
  case 'R':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Wrist", CH_WRIST, STEP_SIZE);
    break;
  case 'F':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Wrist", CH_WRIST, -STEP_SIZE);
    break;
  case 'Z':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Rotate", CH_ROTATE, -STEP_SIZE);
    break;
  case 'X':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Rotate", CH_ROTATE, STEP_SIZE);
    break;
  case 'C':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Gripper", CH_GRIPPER, STEP_SIZE);
    break;
  case 'V':
    if (robotState == STATE_IDLE && !emergencyLocked)
      manualMove("Gripper", CH_GRIPPER, -STEP_SIZE);
    break;
  default:
    Serial.print("Unknown key: ");
    Serial.println(key);
    Serial.println("Press M to show menu.");
    break;
  }
}

void showInitPromptOnce()
{
  if (robotState != STATE_INIT || initPromptShown)
    return;

  Serial.println();
  Serial.println("======================");
  Serial.println(" ROBOT READY CHECK");
  Serial.println("======================");
  Serial.println("Send READY from forest platform or press Y in Serial Monitor.");
  initPromptShown = true;
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup()
{
  Serial.begin(9600);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(100);
  pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);
  delay(500);

  Serial.println();
  Serial.println("Starting Forest Control Deck robot arm...");

  scanI2CBus();
  if (!checkI2CDevice(PCA9685_ADDR))
  {
    Serial.println("ERROR: PCA9685 not detected at 0x40. Check SDA=21, SCL=22, VCC, GND.");
    while (true)
      delay(1000);
  }

  Serial.println("PCA9685 detected.");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);

  initHuskyLens();

  baseAngle = HOME_BASE;
  shoulderAngle = HOME_SHOULDER;
  elbowAngle = HOME_ELBOW;
  wristAngle = HOME_WRIST;
  rotateAngle = HOME_ROTATE;
  gripperAngle = HOME_GRIPPER;
  moveAllCurrentServosUnsafeForStartupOnly();
  delay(800);

  connectWiFiFavoriot();
  testFavoriotConnection();

  Serial.print("MOBILE TRIGGER: watching device ");
  Serial.println(FAVORIOT_MOBILE_ROBOT_DEVICE_ID);
  Serial.println("MOBILE TRIGGER: accepted status = DONE, FINISH, FINISHED, COMPLETE, COMPLETED, or *_complete.");

  printMenu();
  printSafetyStatus();
  changeRobotState(STATE_INIT);
  initPromptShown = false;
}

void loop()
{
  if (checkHardwareEmergency())
    return;

  showInitPromptOnce();

  // Check manual commands from the forest dashboard.
  checkFavoriotDashboardCommand();

  // Check whether the mobile robot has posted a NEW completion status.
  checkMobileRobotCompletionSignal();

  // Start the queued automatic tag scan only when the arm is READY and IDLE.
  runPendingMobileAutoSortIfReady();

  // Check Serial Monitor commands.
  if (Serial.available())
  {
    char key = Serial.read();
    handleSerialKey(key);
  }
}
