/*
  ================================================================
  KS0560 Mecanum Car — COMPRESSED SEQUENCE CONTROL WITH REVERSE
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
#define GRAB_DISTANCE_CM     3

Servo gripperServo;
bool gripperIsOpen = false;

// Color pins
#define COLOR_OUT 6
#define COLOR_S2  7
#define COLOR_S3  8

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


// ── Speeds, Timing & Tuning ───────────────────────────────────
#define SPEED_START_FAST  60   
#define SPEED_START_SLOW  35   //38
#define SPEED_ROTATE      55   //48
#define SPEED_MIN         30   //35
#define CENTER_OFFSET_MS  110  
#define TURN_BLIND_MS     180 //200  
#define QUICK_REVERSE_MS  250  // <-- TIME IN MILLISECONDS TO REVERSE BEFORE TURNING
#define OBJECT_SLOW_DISTANCE_CM  12
#define OBJECT_STOP_DISTANCE_CM   4
#define OBJECT_APPROACH_SPEED    24

#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);
SoftwareSerial espSerial(11, 10);

volatile bool cloudEmergencyStop = false;

// ── SEQUENCE ENGINE DEFINITIONS ───────────────────────────────
enum ActionType {
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_LEFT_MARKERS,
  MOVE_RIGHT_MARKERS,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  DETECT_GRAB,
  DETECT_BLUE_GRAB,
  DETECT_RED_GRAB,
  DETECT_YELLOW_GRAB,
  OPEN_GRIPPER,
  DELAY_MS,
  QUICK_REVERSE,
  REVERSE_TIME 
  
};

struct Step {
  ActionType action;
  int param1;      // Distance / Blocks / Duration
  uint8_t param2;  // Speed
};

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd; speed_Lower_L = spd;
  speed_Upper_R = spd; speed_Lower_R = spd;
}

void setMotorSpeedLR(uint8_t leftSpeed, uint8_t rightSpeed) {
  speed_Upper_L = leftSpeed;
  speed_Lower_L = leftSpeed;

  speed_Upper_R = rightSpeed;
  speed_Lower_R = rightSpeed;
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
  return (duration == 0) ? 999 : duration / 58;
}

void openGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperIsOpen = true;
  delay(500);
}

void closeGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  gripperIsOpen = false;
  delay(700);
}

bool checkEStop()
{
    // Check ESP32 cloud commands during every movement loop.
    // This makes Favoriot STOP work while CMD4/CMD5/CMD6 is running.
    if (espSerial.available())
    {
        String cloudCmd = espSerial.readStringUntil('\n');
        cloudCmd.replace("\r", "");
        cloudCmd.trim();
        cloudCmd.toUpperCase();

        if (cloudCmd == "STOP" ||
            cloudCmd == "STAR" ||
            cloudCmd == "CMDSTAR" ||
            cloudCmd == "EMERGENCY_STOP")
        {
            Serial.println(F("!! CLOUD EMERGENCY STOP !!"));
            cloudEmergencyStop = false;
            car.Stop();
            restoreIR();
            return true;
        }

        // A non-stop cloud command received during a running mission is ignored.
        Serial.print(F("Cloud command ignored while busy: "));
        Serial.println(cloudCmd);
    }

    if (cloudEmergencyStop)
    {
        Serial.println(F("!! CLOUD EMERGENCY STOP !!"));
        cloudEmergencyStop = false;
        car.Stop();
        restoreIR();
        return true;
    }

    // Physical IR star button emergency stop.
    if (IrReceiver.decode())
    {
        if (IrReceiver.decodedIRData.command == CMD_STAR)
        {
            Serial.println(F("!! IR EMERGENCY STOP !!"));
            car.Stop();
            restoreIR();
            return true;
        }

        IrReceiver.resume();
    }

    return false;
}

unsigned long readColorPulse(bool s2, bool s3)
{
    digitalWrite(COLOR_S2, s2);
    digitalWrite(COLOR_S3, s3);
    delay(50);

    unsigned long pulse = pulseIn(COLOR_OUT, LOW, 100000);

    if (pulse == 0)
        pulse = 99999;

    return pulse;
}

String detectColorName()
{
    unsigned long red   = readColorPulse(LOW, LOW);
    unsigned long blue  = readColorPulse(LOW, HIGH);
    unsigned long green = readColorPulse(HIGH, HIGH);

    Serial.print("R=");
    Serial.print(red);
    Serial.print(" G=");
    Serial.print(green);
    Serial.print(" B=");
    Serial.println(blue);

    if (red > 60 && green > 60 && blue > 60)
        return "NO OBJECT";

    if (blue < red && blue < green)
        return "BLUE";

    if (red < blue &&
        green < blue &&
        abs((int)red - (int)green) < 25)
        return "YELLOW";

    if (red < green && red < blue)
        return "RED";

    return "UNKNOWN";
}
bool moveBackwardBlocks(int targetBlocks, uint8_t speed);
bool rotate90(bool turnLeft);

bool reverseForTime(unsigned long durationMs, uint8_t speed)
{
    Serial.println(F("--- Reverse By Time ---"));

    setMotorSpeed(speed);

    unsigned long start = millis();

    while (millis() - start < durationMs)
    {
        if (checkEStop())
        {
            car.Stop();
            return false;
        }

        car.Back();
    }

    car.Stop();
    delay(200);

    restoreIR();

    Serial.println(F("--- Reverse Complete ---"));

    return true;
}

bool DetectBlueAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "BLUE")
        {
          Serial.println(F("BLUE confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not BLUE. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing BLUE."));
  restoreIR();

  return false;
}

bool DetectRedAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "RED")
        {
          Serial.println(F("RED confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not RED. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing RED."));
  restoreIR();

  return false;
}


bool DetectYellowAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "YELLOW")
        {
          Serial.println(F("YELLOW confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not YELLOW. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing YELLOW."));
  restoreIR();

  return false;
}


// ================================================================
//  MOVEMENT CONTROLS
// ================================================================
bool DetectAndGrab(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true;
  openGripper();

  while (junctionCount < targetBlocks) {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();
    if (distance > 0 && distance <= GRAB_DISTANCE_CM) {
      car.Stop();
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
    delay(10);
  }
  car.Stop();
  restoreIR();
  return true;
}

bool rotate90(bool turnLeft) {
  unsigned long t_start = millis();
  uint8_t sensor = turnLeft ? STOP_SENSOR_LEFT_TURN : STOP_SENSOR_RIGHT_TURN;
  
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEStop()) return false;
    setMotorSpeed(SPEED_ROTATE);
    turnLeft ? car.Turn_Left() : car.Turn_Right();
  }
  while (true) {
    if (checkEStop()) return false;
    setMotorSpeed(SPEED_ROTATE);
    turnLeft ? car.Turn_Left() : car.Turn_Right();
    if (digitalRead(sensor) == HIGH) break;
  }
  car.Stop();
  delay(200);
  return true;
}

bool moveLineTracking(ActionType type, int targetBlocks, uint8_t startSpeed) {
  int count = 0;
  bool onMark = true;

  while (count < targetBlocks) {
    if (checkEStop()) return false;

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int decelStep = (type == MOVE_FORWARD) ? 7 : 3;
    int calculatedSpeed = startSpeed - (count * decelStep);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    bool triggered = false;
    if (type == MOVE_FORWARD)       triggered = (L == HIGH && M == HIGH && R == HIGH);
    else if (type == MOVE_LEFT_MARKERS)  triggered = (M == HIGH && L == HIGH);
    else if (type == MOVE_RIGHT_MARKERS) triggered = (M == HIGH && R == HIGH);

    if (triggered) {
      if (!onMark) {
        count++;
        onMark = true;
        if (count == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)  { onMark = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH)  { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW)  { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)   { onMark = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
  }

  if (count == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS);
  }
  restoreIR();
  return true;
}

bool moveBackwardBlocks(int targetBlocks, uint8_t speed) {
  int junctionCount = 0;
  bool onJunction = true;

  const uint8_t correction = 7;
  const uint8_t minimumSideSpeed = 28;

  Serial.print(F("\n--- Reverse "));
  Serial.print(targetBlocks);
  Serial.println(F(" Junctions ---"));

  while (junctionCount < targetBlocks) {
    if (checkEStop()) {
      car.Stop();
      return false;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t slowerSpeed =
      (speed > correction) ? speed - correction : minimumSideSpeed;

    uint8_t fasterSpeed = speed + correction;

    // Full junction
    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Reverse Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks) {
          break;
        }
      }

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Properly centered
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Line detected on right
    // Gently correct left while continuing backward
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;

      setMotorSpeedLR(fasterSpeed, slowerSpeed);
      car.Back();
    }

    // Line detected on left
    // Gently correct right while continuing backward
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(slowerSpeed, fasterSpeed);
      car.Back();
    }

    // No line: keep reversing straight slowly
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Left + middle
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(slowerSpeed, fasterSpeed);
      car.Back();
    }

    // Middle + right
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;

      setMotorSpeedLR(fasterSpeed, slowerSpeed);
      car.Back();
    }

    delay(15);
  }

  // Move backward slightly to leave the junction
  setMotorSpeedLR(speed, speed);
  car.Back();
  delay(100);

  car.Stop();
  delay(300);

  restoreIR();

  Serial.println(F("--- Reverse Complete and Aligned ---"));
  return true;
}

// ================================================================
//  SEQUENCE EXECUTION MOTOR
// ================================================================
void executeSequence(Step sequence[], int totalSteps) {
  for (int i = 0; i < totalSteps; i++) {
    bool stepSuccess = true;
    
    switch (sequence[i].action) {
      case MOVE_FORWARD:
      case MOVE_LEFT_MARKERS:
      case MOVE_RIGHT_MARKERS:
        stepSuccess = moveLineTracking(sequence[i].action, sequence[i].param1, sequence[i].param2);
        break;
        case MOVE_BACKWARD:
        stepSuccess = moveBackwardBlocks(
          sequence[i].param1,
          sequence[i].param2
        );
        break;
      case ROTATE_LEFT:
        stepSuccess = rotate90(true);
        break;
      case ROTATE_RIGHT:
        stepSuccess = rotate90(false);
        break;
      case DETECT_GRAB:
        stepSuccess = DetectAndGrab(sequence[i].param1, sequence[i].param2);
        break;
        case DETECT_BLUE_GRAB:
        stepSuccess = DetectBlueAndGrab(
            sequence[i].param1,
            sequence[i].param2
        );
        break;
      case DETECT_RED_GRAB:
        stepSuccess = DetectRedAndGrab(
            sequence[i].param1,
            sequence[i].param2
        );
        break;
      case DETECT_YELLOW_GRAB:
        stepSuccess = DetectYellowAndGrab(
            sequence[i].param1,
            sequence[i].param2
        );
        break;
      case OPEN_GRIPPER:
        openGripper();
        break;
      case DELAY_MS:
        delay(sequence[i].param1);
        break;
      case QUICK_REVERSE:
        setMotorSpeed(sequence[i].param2); // Uses assigned speed parameter
        car.Back();
        delay(sequence[i].param1);         // Backs up for assigned duration parameter
        car.Stop();
        break;
        case REVERSE_TIME:
        stepSuccess = reverseForTime(
        sequence[i].param1,   // milliseconds
        sequence[i].param2    // speed
    );
    break;
        
    }
    
    if (!stepSuccess) {
      Serial.println(F("Sequence Termination triggered."));
      break; 
    }
  }
  car.Stop();
}

void runBlueSuccessPath1() {
  Step successPath1[] = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath1,
    sizeof(successPath1) / sizeof(successPath1[0])
  );
}

void runBlueSuccessPath2() {
  Step successPath2[] = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath2,
    sizeof(successPath2) / sizeof(successPath2[0])
  );
}

void runBlueSuccessPath3() {
  Step successPath3[] = {
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},
    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath3,
    sizeof(successPath3) / sizeof(successPath3[0])
  );
}

// Go for Left Side
void goFromMiddleToLeftSide() {
  Step goToLeftSide[] = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToLeftSide,
    sizeof(goToLeftSide) / sizeof(goToLeftSide[0])
  );
}

void goFromLeftToRightSide() {
  Step goToRightSide[] = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 4, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToRightSide,
    sizeof(goToRightSide) / sizeof(goToRightSide[0])
  );
}

bool runCmd2MiddlePath()
{
    Step middlePath[] = {
        {DETECT_GRAB, 6, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 7, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        middlePath,
        sizeof(middlePath) / sizeof(middlePath[0])
    );

    return true;
}

bool runCmd2LeftPath()
{
    Step leftPath[] = {
        {MOVE_FORWARD, 3, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {DETECT_GRAB, 3, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 7, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        leftPath,
        sizeof(leftPath) / sizeof(leftPath[0])
    );

    return true;
}

bool runCmd2RightPath()
{
    Step rightPath[] = {
        {MOVE_FORWARD, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {DETECT_GRAB, 4, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 7, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        rightPath,
        sizeof(rightPath) / sizeof(rightPath[0])
    );

    return true;
}

void runRemote2Mission()
{
    Serial.println(F("Starting Remote 2 mission."));

    runCmd2MiddlePath();
    runCmd2LeftPath();
    runCmd2RightPath();

    car.Stop();
    restoreIR();

    Serial.println(F("Remote 2 mission complete."));
}

void runRemote4Mission()
{
  Serial.println(F("Starting Remote 4 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  // Step approachMiddle[] = {
  //   {MOVE_FORWARD, 3, SPEED_START_SLOW},
  //   {DELAY_MS, 500, 0}
  // };

  // executeSequence(
  //   approachMiddle,
  //   sizeof(approachMiddle) / sizeof(approachMiddle[0])
  // );

  bool firstBlueGrabbed = DetectBlueAndGrab(
    6,
    SPEED_START_SLOW
  );

  if (firstBlueGrabbed) {
    Serial.println(F("Blue found at middle position."));
    runBlueSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not blue. Going left."));

  goFromMiddleToLeftSide();

  bool secondBlueGrabbed = DetectBlueAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondBlueGrabbed) {
    Serial.println(F("Blue found at left position."));
    runBlueSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not blue. Going right."));

  goFromLeftToRightSide();

  bool thirdBlueGrabbed = DetectBlueAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdBlueGrabbed) {
    Serial.println(F("Blue found at right position."));
    runBlueSuccessPath3();
    return;
  }

  Serial.println(F("No blue object found in all 3 positions."));
  car.Stop();
  restoreIR();
}
void runRedSuccessPath1() {
  Step successPath1[] = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

     {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath1,
    sizeof(successPath1) / sizeof(successPath1[0])
  );
}

void runRedSuccessPath2() {
  Step successPath2[] = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath2,
    sizeof(successPath2) / sizeof(successPath2[0])
  );
}

void runRedSuccessPath3() {
  Step successPath3[] = {
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath3,
    sizeof(successPath3) / sizeof(successPath3[0])
  );
}

// Go for Left Side
void goFromMiddleToLeftSideRed() {
  Step goToLeftSide[] = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToLeftSide,
    sizeof(goToLeftSide) / sizeof(goToLeftSide[0])
  );
}

void goFromLeftToRightSideRed() {
  Step goToRightSide[] = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 4, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToRightSide,
    sizeof(goToRightSide) / sizeof(goToRightSide[0])
  );
}

void runRemote5Mission()
{
  Serial.println(F("Starting Remote 5 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  // Step approachMiddle[] = {
  //   {MOVE_FORWARD, 3, SPEED_START_SLOW},
  //   {DELAY_MS, 500, 0}
  // };

  // executeSequence(
  //   approachMiddle,
  //   sizeof(approachMiddle) / sizeof(approachMiddle[0])
  // );

  bool firstRedGrabbed = DetectRedAndGrab(
    6,
    SPEED_START_SLOW
  );

  if (firstRedGrabbed) {
    Serial.println(F("Red found at middle position."));
    runRedSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not red. Going left."));

  goFromMiddleToLeftSideRed();

  bool secondRedGrabbed = DetectRedAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondRedGrabbed) {
    Serial.println(F("Red found at left position."));
    runRedSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not red. Going right."));

  goFromLeftToRightSideRed();

  bool thirdRedGrabbed = DetectRedAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdRedGrabbed) {
    Serial.println(F("Red found at right position."));
    runRedSuccessPath3();
    return;
  }

  Serial.println(F("No red object found in all 3 positions."));
  car.Stop();
  restoreIR();
}

void runRemote6Mission()
{
  Serial.println(F("Starting Remote 6 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  // Step approachMiddle[] = {
  //   {MOVE_FORWARD, 3, SPEED_START_SLOW},
  //   {DELAY_MS, 500, 0}
  // };

  // executeSequence(
  //   approachMiddle,
  //   sizeof(approachMiddle) / sizeof(approachMiddle[0])
  // );

  bool firstYellowGrabbed = DetectYellowAndGrab(
    6,
    SPEED_START_SLOW
  );

  if (firstYellowGrabbed) {
    Serial.println(F("Yellow found at middle position."));
    runRedSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not yellow. Going left."));

  goFromMiddleToLeftSideRed();

  bool secondYellowGrabbed = DetectYellowAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondYellowGrabbed) {
    Serial.println(F("Yellow found at left position."));
    runRedSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not yellow. Going right."));

  goFromLeftToRightSideRed();

  bool thirdYellowGrabbed = DetectYellowAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdYellowGrabbed) {
    Serial.println(F("Yellow found at right position."));
    runRedSuccessPath3();
    return;
  }

  Serial.println(F("No yellow object found in all 3 positions."));
  car.Stop();
  restoreIR();
}

void readCloudCommand()
{
    if (!espSerial.available())
        return;

    String cmd = espSerial.readStringUntil('\n');

    cmd.trim();
    cmd.toUpperCase();

    Serial.print(F("Cloud Command: "));
    Serial.println(cmd);

    if (cmd == "CMD4")
    {
        Serial.println(F("Cloud Start Remote4"));
        runRemote4Mission();
    }
    else if (cmd == "CMD5")
    {
        Serial.println(F("Cloud Start Remote5"));
        runRemote5Mission();
    }
    else if (cmd == "CMD6")
    {
        Serial.println(F("Cloud Start Remote6"));
        runRemote6Mission();
    }
    else if (
        cmd == "STOP" ||
        cmd == "STAR" ||
        cmd == "CMDSTAR" ||
        cmd == "EMERGENCY_STOP"
    )
    {
        Serial.println(F("!!! CLOUD EMERGENCY STOP !!!"));
        cloudEmergencyStop = false;
        car.Stop();
        restoreIR();
    }
}




// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  espSerial.setTimeout(50);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  gripperServo.attach(GRIPPER_PIN);
  openGripper();

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready."));
  espSerial.println(F("UNO_READY"));
}

void loop()
{
    // Receive cloud commands while the robot is idle.
    readCloudCommand();

    if (!IrReceiver.decode())
        return;

    uint8_t cmd = IrReceiver.decodedIRData.command;

    if (
        cmd == 0x00 ||
        (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)
    )
    {
        IrReceiver.resume();
        return;
    }

    switch (cmd)
    {
        case CMD_1:
        {
            Step path1[] = {
                {MOVE_FORWARD, 3, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DETECT_GRAB, 3, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_FORWARD, 7, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {OPEN_GRIPPER, 0, 0},
                {MOVE_BACKWARD, 1, SPEED_START_SLOW}
            };

            executeSequence(
                path1,
                sizeof(path1) / sizeof(path1[0])
            );
            break;
        }

        case CMD_2:
            runRemote2Mission();
            break;
        

        case CMD_3:
        {
            Step path3[] = {
                {MOVE_FORWARD, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DETECT_GRAB, 4, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_FORWARD, 7, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {OPEN_GRIPPER, 0, 0},
                {MOVE_BACKWARD, 1, SPEED_START_SLOW}
            };

            executeSequence(
                path3,
                sizeof(path3) / sizeof(path3[0])
            );
            break;
        }

        case CMD_4:
            runRemote4Mission();
            break;

        case CMD_5:
            runRemote5Mission();
            break;

        case CMD_6:
            runRemote6Mission();
            break;

        case CMD_STAR:
            Serial.println(F("!! EMERGENCY STOP !!"));
            cloudEmergencyStop = false;
            car.Stop();
            restoreIR();
            break;
    }

    IrReceiver.resume();
}

