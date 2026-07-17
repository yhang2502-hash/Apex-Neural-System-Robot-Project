/*
  ================================================================
  KS0560 Mecanum Car — FAST ROTATION + SLOW POST-TURN MOVEMENT
  ================================================================
  LOGIC: 
    - Button 1: Move 5 blocks forward starting at 60.
    - Button 2: Rotate Right at 60, then move 2 blocks starting at 40.
    - Button 3: Rotate Left at 60, then move 3 right-markers starting at 40.
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
#define SPEED_START_FAST  60   // Normal forward start speed (Button 1)
#define SPEED_START_SLOW  35   // Forward speed used after rotating (Button 2 & 3)
#define SPEED_ROTATE      48   // Maintained strong speed for 90-degree rotations
#define SPEED_MIN         35   // Safety floor so speed never hits 0
#define CENTER_OFFSET_MS  110    // ms to drive forward after final junction
#define TURN_BLIND_MS     150  // Blind turn duration to clear the starting line

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);
SoftwareSerial espSerial(11, 10);
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

  if (duration == 0)
    return 999;

  return duration / 58;
}

void openGripper() {
  if (!gripperServo.attached())
    gripperServo.attach(GRIPPER_PIN);

  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperIsOpen = true;
  delay(500);
}

void closeGripper() {
  if (!gripperServo.attached())
    gripperServo.attach(GRIPPER_PIN);

  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  gripperIsOpen = false;
  delay(700);
}

bool DetectAndGrab(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks) {
    long distance = readUltrasonicDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 0 && distance <= GRAB_DISTANCE_CM) {
      car.Stop();
      Serial.println("Object Found! Closing gripper...");
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
        Serial.print("Junction Crossed: ");
        Serial.println(junctionCount);
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        restoreIR();
        return false;
      }
      IrReceiver.resume();
    }

    delay(10);
  }

  car.Stop();
  Serial.println("Reached 6 junctions. No object grabbed.");
  restoreIR();
  return false;
}
// ================================================================
//  TURN 90 DEGREES RIGHT (Maintains Speed 60)
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Speed 60) ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE); // Maintained at 60
    car.Turn_Right();          
  }

  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();

    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
      Serial.println(F("Line found! Stopping turn."));
      break;
    }
  }

  car.Stop();
  delay(200); 
  return true;
}

// ================================================================
//  TURN 90 DEGREES LEFT (Maintains Speed 60)
// ================================================================
bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left (Speed 60) ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE); // Maintained at 60
    car.Turn_Left();
  }

  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();

    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
      Serial.println(F("Line found! Stopping turn."));
      break;
    }
  }

  car.Stop();
  delay(200); 
  return true;
}



// ================================================================
//  UNIVERSAL FORWARD MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true; 

  Serial.print(F("\n--- Moving Forward "));
  Serial.print(targetBlocks);
  Serial.print(F(" Blocks (Starting at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(") ---"));

  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    // --- DYNAMIC DECELERATION FORMULA ---
    int calculatedSpeed = startSpeed - (junctionCount * 7);
    if (calculatedSpeed < SPEED_MIN) {
      calculatedSpeed = SPEED_MIN;
    }
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        Serial.print(F("Junction Crossed: "));
        Serial.print(junctionCount);
        Serial.print(F(" | Speed: "));
        Serial.println(calculatedSpeed);
        onJunction = true; 
        if (junctionCount == targetBlocks) break; 
      }
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false; 
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);      
      car.Turn_Right();               
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);      
      car.Turn_Left();                
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP PRESSED!"));
        break; 
      }
      IrReceiver.resume();
    }
  }

  if (junctionCount == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN);
    car.Advance();
    delay(CENTER_OFFSET_MS); 
  }

  Serial.println(F("--- Movement Completed. ---"));
  restoreIR(); 
}
// ================================================================
//  LEFT-SENSOR BIAS MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveLeftSideMarkers(int targetBlocks, uint8_t startSpeed) {
  int markerCount = 0;
  bool onMarker = true; 

  Serial.print(F("\n--- Watching Left Sensor for "));
  Serial.print(targetBlocks);
  Serial.print(F(" Markers (Starting at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(") ---"));

  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    // --- DYNAMIC DECELERATION FORMULA ---
    int calculatedSpeed = startSpeed - (markerCount * 3);
    if (calculatedSpeed < SPEED_MIN) {
      calculatedSpeed = SPEED_MIN;
    }
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (M == HIGH && L == HIGH) {
      if (!onMarker) {
        markerCount++;
        Serial.print(F("LEFT Marker Crossed: "));
        Serial.print(markerCount);
        Serial.print(F(" | Speed: "));
        Serial.println(calculatedSpeed);
        onMarker = true; 
        if (markerCount == targetBlocks) break; 
      }
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)  { onMarker = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP PRESSED!"));
        break; 
      }
      IrReceiver.resume();
    }
  }

  if (markerCount == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN);
    car.Advance();
    delay(CENTER_OFFSET_MS); 
  }

  Serial.println(F("--- LEFT Marker Movement Completed. ---"));
  restoreIR(); 
}

// ================================================================
//  RIGHT-SENSOR BIAS MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveRightSideMarkers(int targetBlocks, uint8_t startSpeed) {
  int markerCount = 0;
  bool onMarker = true; 

  Serial.print(F("\n--- Watching Right Sensor for "));
  Serial.print(targetBlocks);
  Serial.print(F(" Markers (Starting at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(") ---"));

  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    // --- DYNAMIC DECELERATION FORMULA ---
    int calculatedSpeed = startSpeed - (markerCount * 3);
    if (calculatedSpeed < SPEED_MIN) {
      calculatedSpeed = SPEED_MIN;
    }
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (M == HIGH && R == HIGH) {
      if (!onMarker) {
        markerCount++;
        Serial.print(F("Right Marker Crossed: "));
        Serial.print(markerCount);
        Serial.print(F(" | Speed: "));
        Serial.println(calculatedSpeed);
        onMarker = true; 
        if (markerCount == targetBlocks) break; 
      }
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) {
      onMarker = false; 
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);      
      car.Turn_Right();               
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);      
      car.Turn_Left();                
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed);  
      car.Advance();                  
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP PRESSED!"));
        break; 
      }
      IrReceiver.resume();
    }
  }

  if (markerCount == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN);
    car.Advance();
    delay(CENTER_OFFSET_MS); 
  }

  Serial.println(F("--- Right Marker Movement Completed. ---"));
  restoreIR(); 
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  gripperServo.attach(GRIPPER_PIN);
  openGripper();

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready."));
  Serial.println(F("Press '1' to go 7 blocks forward."));
  Serial.println(F("Press '2' to turn right and go 2 blocks."));
  Serial.println(F("Press '3' to turn left and go 3 right-markers."));
}

void loop() {
  if (!IrReceiver.decode()) return;

  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    IrReceiver.resume(); 
    return;
  }

  if (cmd == CMD_1) {
    moveForwardBlocks(3, SPEED_START_SLOW); // Starts straight out at 60
     if (rotateLeft90()) { 
      moveRightSideMarkers(2, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!    
    } 
    if(rotateRight90()){
      DetectAndGrab(3,SPEED_START_SLOW);
    }
    rotateRight90();
    delay(500);
    if(rotateRight90()){
      delay(500);
      moveForwardBlocks(3,SPEED_START_SLOW);
    }
    if(rotateLeft90()){
      delay(500);
      moveLeftSideMarkers(2,SPEED_START_SLOW);
    }
    if(rotateRight90()){
      delay(500);
      moveRightSideMarkers(4, SPEED_START_SLOW);
      openGripper();
    
  }} else if (cmd == CMD_2) {
    DetectAndGrab(6, SPEED_START_SLOW);
    rotateRight90();
    delay(500);
    if(rotateRight90()) {
        delay(500);
      moveForwardBlocks(7,SPEED_START_SLOW);
      openGripper();
    }
  
  } else if (cmd == CMD_3) {
    moveForwardBlocks(3, SPEED_START_SLOW); // Starts straight out at 60
     if (rotateRight90()) { 
      moveLeftSideMarkers(2, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!    
    } 
    if(rotateLeft90()){
      DetectAndGrab(3,SPEED_START_SLOW);
    }
    if(rotateLeft90()){
      moveForwardBlocks(3,SPEED_START_SLOW);
    };
    delay(500);
    if(rotateLeft90()){
      delay(500);
      moveForwardBlocks(5,SPEED_START_SLOW);
      delay(200);
      openGripper();
    }
    }
    else if (cmd == CMD_4) {
    moveForwardBlocks(3, SPEED_START_SLOW); // Starts straight out at 60
    delay(500);
     if (rotateRight90()) { 
      delay(500);
      moveRightSideMarkers(2, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!    
      delay(500);
    } 
    if(rotateLeft90()){
      delay(500);
      DetectAndGrab(3,SPEED_START_SLOW);
    }
    rotateLeft90();
    delay(500);
    if(rotateLeft90()){
      delay(500);
      moveForwardBlocks(3,SPEED_START_SLOW);
      delay(500);
    }
    if(rotateRight90()){
      delay(500);
      moveRightSideMarkers(2,SPEED_START_SLOW);
      delay(500);
    }
    if(rotateLeft90()){
      delay(500);
      moveRightSideMarkers(4, SPEED_START_SLOW);
      delay(500);
      openGripper();
    
  }}

  IrReceiver.resume();
}