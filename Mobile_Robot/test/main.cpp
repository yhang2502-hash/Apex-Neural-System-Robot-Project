/*
  ================================================================
  KS0560 Mecanum Car — FAST ROTATION + SLOW POST-TURN MOVEMENT
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>
#include <Servo.h>
#include <SoftwareSerial.h>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

// ── Ultrasonic + Gripper pins ─────────────────────────────────
#define TRIG_PIN        12
#define ECHO_PIN        13
#define GRIPPER_PIN     9

#define GRIPPER_OPEN_ANGLE   30
#define GRIPPER_CLOSE_ANGLE  100
#define GRAB_DISTANCE_CM     4

Servo gripperServo;
bool gripperIsOpen = false;

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── IR Codes ──────────────────────────────────────────────────
#define CMD_1           0x16
#define CMD_2           0x19
#define CMD_3           0x0D
#define CMD_4           0x0C
#define CMD_5           0x18
#define CMD_6           0x5E
#define CMD_7           0x08
#define CMD_8           0x1C  
#define CMD_STAR        0x42

// ── TCS3200 Color Sensor Pins ────────────────────────────────
#define COLOR_S0   6
#define COLOR_S1   5
#define COLOR_S2   4
#define COLOR_S3   3
#define COLOR_OUT  2
#define COLOR_OE   7
#define COLOR_LED  8

// ── Speeds & Timing ───────────────────────────────────────────
#define SPEED_START_FAST  55   
#define SPEED_START_SLOW  40   
#define SPEED_ROTATE      55   
#define SPEED_MIN         35   
#define CENTER_OFFSET_MS  110    
#define TURN_BLIND_MS     150  

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);
SoftwareSerial espSerial(11, 10); // RX = 11, TX = 10 -> Connects to ESP32 Pin 16 (RXD2)

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd;
  speed_Lower_L = spd;
  speed_Upper_R = spd;
  speed_Lower_R = spd;
}

void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

long readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration / 58;
}

// Fixed function to cleanly forward data to ESP32 over SoftwareSerial
void sendDistanceToESP32(long distance) {
  espSerial.flush();
  delay(10);
  espSerial.print(distance);
  espSerial.print('\n'); // Critical termination character
}

void openGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperIsOpen = true;
  Serial.println(F("Gripper OPEN"));
  delay(500);
}

void closeGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  gripperIsOpen = false;
  Serial.println(F("Gripper CLOSE"));
  delay(700);
}

// ================================================================
//  ROTATIONS
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Speed 60) ---"));
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) { restoreIR(); return false; }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();          
  }
  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) { restoreIR(); return false; }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) break;
  }
  car.Stop();
  delay(200); 
  return true;
}

bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left (Speed 60) ---"));
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) { restoreIR(); return false; }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
  }
  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) { restoreIR(); return false; }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) break;
  }
  car.Stop();
  delay(200); 
  return true;
}

// ================================================================
//  MOVEMENT MOTIONS
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true; 
  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);
    int calculatedSpeed = startSpeed - (junctionCount * 7);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true; 
        if (junctionCount == targetBlocks) break; 
      }
      setMotorSpeed(currentSpeed); car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)  { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }

    if (IrReceiver.decode()) { if (IrReceiver.decodedIRData.command == CMD_STAR) break; IrReceiver.resume(); }
  }
  if (junctionCount == targetBlocks && CENTER_OFFSET_MS > 0) { setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS); }
  restoreIR(); 
}

void moveLeftSideMarkers(int targetBlocks, uint8_t startSpeed) {
  int markerCount = 0;
  bool onMarker = true; 
  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);
    int calculatedSpeed = startSpeed - (markerCount * 2);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH) {
      if (!onMarker) { markerCount++; onMarker = true; if (markerCount == targetBlocks) break; }
      setMotorSpeed(currentSpeed); car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)  { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }

    if (IrReceiver.decode()) { if (IrReceiver.decodedIRData.command == CMD_STAR) break; IrReceiver.resume(); }
  }
  if (markerCount == targetBlocks && CENTER_OFFSET_MS > 0) { setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS); }
  restoreIR(); 
}

void moveRightSideMarkers(int targetBlocks, uint8_t startSpeed) {
  int markerCount = 0;
  bool onMarker = true; 
  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);
    int calculatedSpeed = startSpeed - (markerCount * 2);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (M == HIGH && R == HIGH) {
      if (!onMarker) { markerCount++; onMarker = true; if (markerCount == targetBlocks) break; }
      setMotorSpeed(currentSpeed); car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)  { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }

    if (IrReceiver.decode()) { if (IrReceiver.decodedIRData.command == CMD_STAR) break; IrReceiver.resume(); }
  }
  if (markerCount == targetBlocks && CENTER_OFFSET_MS > 0) { setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS); }
  restoreIR(); 
}

// ================================================================
//  ULTRASONIC GRAB FUNCTIONS
// ================================================================
bool moveForwardAndGrabObject(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true;
  unsigned long lastSendTime = 0;

  Serial.println(F("\nRobot started. Reading distance and sending to ESP32..."));
  openGripper();

  while (junctionCount < targetBlocks) {
    long distance = readUltrasonicDistance();

    // Limit transmission interval to prevent serial floods
    if (millis() - lastSendTime > 300) { 
      Serial.print(F("Local Distance: ")); Serial.print(distance); Serial.println(F(" cm"));
      sendDistanceToESP32(distance); // Sends clean integer to ESP32
      lastSendTime = millis();
    }

    if (distance > 0 && distance <= GRAB_DISTANCE_CM) {
      car.Stop();
      Serial.println(F("Object detected <= 4cm. Grabbing object."));
      closeGripper();
      restoreIR();
      return true;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);
    int calculatedSpeed = startSpeed - (junctionCount * 7);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)  { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH)  { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW)  { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)   { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) { restoreIR(); return false; }
      IrReceiver.resume();
    }
    delay(10);
  }

  car.Stop();
  restoreIR();
  return false;
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  espSerial.begin(9600);
  Serial.begin(9600);

  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);


  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  pinMode(COLOR_OE, OUTPUT);
  pinMode(COLOR_LED, OUTPUT);

  digitalWrite(COLOR_OE, LOW);    // enable sensor
  digitalWrite(COLOR_LED, HIGH);   // official Keyestudio style
  digitalWrite(COLOR_S0, LOW);    // 2% frequency scaling
  digitalWrite(COLOR_S1, HIGH);

  car.Init();
  IrReceiver.begin(RECV_PIN, false);
  Serial.println(F("Ready."));
}
unsigned long readColorPulse(bool s2, bool s3) {
  digitalWrite(COLOR_S2, s2);
  digitalWrite(COLOR_S3, s3);
  delay(50);

  unsigned long pulse = pulseIn(COLOR_OUT, LOW, 50000);
  if (pulse == 0) pulse = 99999;
  return pulse;
}

String detectColorName() {
  // For TCS3200: smaller pulse usually means stronger color
  unsigned long redPulse   = readColorPulse(LOW, LOW);
  unsigned long bluePulse  = readColorPulse(LOW, HIGH);
  unsigned long greenPulse = readColorPulse(HIGH, HIGH);

  Serial.print(F("R pulse: "));
  Serial.print(redPulse);
  Serial.print(F(" | G pulse: "));
  Serial.print(greenPulse);
  Serial.print(F(" | B pulse: "));
  Serial.println(bluePulse);

  if (redPulse < bluePulse && redPulse < greenPulse) {
    if (greenPulse < bluePulse) {
      return "YELLOW";
    }
    return "RED";
  }

  if (bluePulse < redPulse && bluePulse < greenPulse) {
    return "BLUE";
  }

  return "UNKNOWN";
}

void testColorSensor() {
  Serial.println(F("\n=== COLOR SENSOR TEST ==="));
  String color = detectColorName();

  Serial.print(F("Detected Color: "));
  Serial.println(color);
}

void loop() {
  if (!IrReceiver.decode()) return;
  uint8_t cmd = IrReceiver.decodedIRData.command;
  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) { IrReceiver.resume(); return; }

  if (cmd == CMD_5) {
    moveForwardAndGrabObject(7, SPEED_START_SLOW);
    rotateRight90();
    if (rotateRight90()) { moveForwardBlocks(6, SPEED_START_SLOW); }
    openGripper();
  } 
  else if (cmd == CMD_1) {
  testColorSensor();
}
  else if (cmd == CMD_2) {
    if (rotateRight90()) { moveForwardBlocks(2, SPEED_START_SLOW); }
  } 
  else if (cmd == CMD_3) {
    if (rotateLeft90()) { moveRightSideMarkers(2, SPEED_START_SLOW); } 
  } 
  else if (cmd == CMD_6) {
    openGripper();
    moveForwardBlocks(3, SPEED_START_FAST);
    if (rotateRight90()) { moveRightSideMarkers(2, SPEED_START_SLOW); }
    if (rotateLeft90())  { moveRightSideMarkers(1, SPEED_START_SLOW); moveForwardAndGrabObject(3, SPEED_START_SLOW); }
    if (rotateLeft90())  { if (rotateLeft90()) { moveLeftSideMarkers(3, SPEED_START_SLOW); } }
    if (rotateRight90()) { moveRightSideMarkers(2, SPEED_START_SLOW); if (rotateLeft90()) { moveLeftSideMarkers(4, SPEED_START_SLOW); openGripper(); } }
  } 
  else if (cmd == CMD_4) {
    openGripper();
    moveForwardBlocks(3, SPEED_START_FAST);
    if (rotateLeft90())  { moveLeftSideMarkers(2, SPEED_START_SLOW); }
    if (rotateRight90()) { moveLeftSideMarkers(1, SPEED_START_SLOW); moveForwardAndGrabObject(3, SPEED_START_SLOW); }
    if (rotateRight90()) { if (rotateRight90()) { moveRightSideMarkers(3, SPEED_START_SLOW); } }
    if (rotateLeft90())  { moveLeftSideMarkers(2, SPEED_START_SLOW); if (rotateRight90()) { moveRightSideMarkers(4, SPEED_START_SLOW); openGripper(); } }
  }

  IrReceiver.resume();
}