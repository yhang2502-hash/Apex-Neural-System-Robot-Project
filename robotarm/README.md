# Robotic Arm

The robotic arm is the sorting and transfer unit of the **Apex Neural System Robot Project**.

After the mobile robot reaches the dropping area, the robotic arm receives a task trigger, detects the required object or tag, and executes the corresponding pick-and-place sequence.

[← Back to Main Project](../README.md)

---

## Overview

The robotic-arm system combines:

- ESP32 control
- PCA9685 servo control
- HuskyLens recognition
- Pre-recorded movement sequences
- Favoriot cloud communication
- A Node.js web-control platform

The arm is designed to receive the object delivered by the mobile robot and place it into the correct coloured destination container.

---

## Main Responsibilities

The robotic arm is responsible for:

1. Moving to the home position
2. Connecting to Wi-Fi and Favoriot
3. Waiting for the mobile robot to complete delivery
4. Receiving a completion status
5. Reading the HuskyLens result
6. Selecting the correct pick-and-place sequence
7. Gripping the object
8. Moving it to the correct container
9. Releasing the object
10. Returning to the home position

---

## Collaboration Workflow

```text
Mobile Robot Reaches Dropping Area
               ↓
Completion Status Is Sent
               ↓
Robotic Arm Receives Trigger
               ↓
HuskyLens Reads Tag or Object
               ↓
Correct Sequence Is Selected
               ↓
Gripper Picks Up Object
               ↓
Arm Moves to Destination Box
               ↓
Object Is Released
               ↓
Arm Returns Home
```

---

## Demonstration Scenario

The mobile robot transports a coloured object to the dropping area beside the robotic arm.

The arm remains at its home position until a valid task status is received.

After receiving the trigger, the ESP32 checks the HuskyLens recognition result or command value. The controller then selects one of the stored movement sequences.

For example:

```text
Tag ID 1 → Blue sorting sequence
Tag ID 2 → Red sorting sequence
Tag ID 3 → Yellow sorting sequence
```

The arm approaches the object, closes the gripper, lifts it, rotates towards the correct container, releases it, and returns to the standby position.

---

## Hardware

The robotic-arm system uses:

- Six-degree-of-freedom robotic arm
- ESP32 development board
- PCA9685 PWM servo driver
- HuskyLens AI vision sensor
- Six servo motors
- Robotic gripper
- External servo power supply
- USB connection for programming
- Coloured objects
- Blue, red and yellow destination containers

---

## Servo Joint Functions

The robotic arm contains six servo-controlled movements:

| Joint | Function |
|---|---|
| Base | Rotates the arm left and right |
| Shoulder | Raises or lowers the main arm section |
| Elbow | Extends or retracts the arm |
| Wrist | Adjusts the end-effector angle |
| Wrist rotation | Rotates the gripper |
| Gripper | Opens and closes to hold an object |

The PCA9685 driver generates PWM signals for all servo motors.

---

## HuskyLens Recognition

The HuskyLens sensor is used to detect the required tag or object category.

The project uses HuskyLens through UART communication.

A typical connection may use:

```text
ESP32 RX2 → HuskyLens TX
ESP32 TX2 → HuskyLens RX
GND       → GND
Power     → Supported HuskyLens supply
```

The exact ESP32 pins should match the firmware configuration.

### Tag Mapping

The recognition mapping can be configured as:

```text
Tag 1 → Blue object
Tag 2 → Red object
Tag 3 → Yellow object
```

When a valid tag is detected, the arm selects the matching movement sequence.

---

## Pick-and-Place Sequence

Each colour sequence contains several stored servo positions.

A typical sequence is:

1. Move to the home position
2. Rotate towards the dropping area
3. Lower the shoulder
4. Adjust the elbow
5. Position the wrist
6. Open the gripper
7. Approach the object
8. Close the gripper
9. Lift the object
10. Rotate towards the destination container
11. Lower the object
12. Open the gripper
13. Move away from the container
14. Return to the home position

---

## Smooth Servo Movement

Instead of moving each servo directly from one angle to another, the firmware can gradually change the servo position.

This produces smoother movement:

```text
Current Position
      ↓
Small Angle Increment
      ↓
Short Time Interval
      ↓
Next Small Increment
      ↓
Target Position
```

This method reduces:

- Sudden movement
- Servo shaking
- Object dropping
- Excessive mechanical stress
- Collision risk

---

## Recorded and Hard-Coded Sequences

The firmware supports both recorded and hard-coded operations.

### Recorded Sequence

The user manually positions the arm and stores each pose.

Example workflow:

```text
Move arm manually
      ↓
Press N to record the pose
      ↓
Move to next position
      ↓
Record again
      ↓
Play the complete sequence
```

### Hard-Coded Sequence

Servo angles and delay values are stored directly inside arrays in the source code.

Example structure:

```cpp
ActionStep sequence[] = {
    {base, shoulder, elbow, wrist, rotate, gripper, delay},
    {base, shoulder, elbow, wrist, rotate, gripper, delay}
};
```

Hard-coded sequences provide repeatable movements for the demonstration.

---

## Keyboard Controls

The robotic arm can be manually controlled through the Serial Monitor.

| Key | Function |
|---|---|
| W / S | Move shoulder |
| A / D | Rotate base |
| Q / E | Move elbow |
| R / F | Move wrist |
| Z / X | Rotate wrist |
| C / V | Open or close gripper |
| H | Return to home position |
| K | Emergency stop |
| N | Record the current pose |
| Y | Play recorded sequence |
| O | Loop recorded sequence |
| P | Play hard-coded sequence |
| 2 | Loop hard-coded sequence |
| L | Read HuskyLens result |

The exact key functions may vary according to the firmware version.

---

## Emergency Stop

Pressing the emergency-stop key stops the current movement sequence.

The safety logic should:

1. Stop sequence execution
2. Prevent new automatic tasks
3. Keep the current servo position where possible
4. Wait for a recovery command
5. Allow the home command after the danger has been cleared

The emergency-stop function should always be tested before running an automatic sequence.

---

## Servo Safety Limits

Each joint has a permitted angle range.

Example safety controls include:

- Base angle limit
- Shoulder minimum and maximum angle
- Elbow movement limit
- Wrist angle limit
- Gripper range
- Protected zone around the HuskyLens
- Shoulder and elbow safety margins

These limits help prevent:

- Servo over-rotation
- Collision with the table
- Collision with the camera
- Mechanical damage
- Unsafe object movement

---

## Camera Protection Zone

The HuskyLens is mounted close to the arm.

The program uses a protected zone to prevent the shoulder, elbow or wrist from colliding with the sensor.

The protection logic should check the combined position of several joints instead of checking only one servo.

This prevents one joint from bypassing the safety limit by moving through another direction.

---

## Favoriot Communication

The robotic arm communicates with the Favoriot IoT platform through Wi-Fi.

The system can:

- Read the mobile robot's completion status
- Receive web-dashboard commands
- Send robot-arm status
- Report sequence completion
- Support automatic sorting commands

### Example Commands

```text
READY
STATUS
BLUE
RED
YELLOW
AUTO_SORT
```

### Example Mobile Robot Status

```text
DONE
FINISHED
COMPLETED
TaskMiddle_complete
TaskLeft_complete
TaskRight_complete
```

---

## Private Configuration

Private credentials must not be written directly inside `main.cpp`.

Create:

```text
include/secrets.h
```

Example:

```cpp
#pragma once

static const char WIFI_SSID[] =
    "YOUR_WIFI_NAME";

static const char WIFI_PASSWORD[] =
    "YOUR_WIFI_PASSWORD";

static const char FAVORIOT_API_KEY[] =
    "YOUR_FAVORIOT_API_KEY";

static const char FAVORIOT_COMMAND_DEVICE_ID[] =
    "YOUR_COMMAND_DEVICE_ID";

static const char FAVORIOT_MOBILE_ROBOT_DEVICE_ID[] =
    "YOUR_MOBILE_ROBOT_DEVICE_ID";
```

The `secrets.h` file is ignored by Git and must not be uploaded.

A public template may be provided as:

```text
include/secrets.example.h
```

---

## Web Control Platform

The user-interface platform is located at:

```text
User Interface Part/web_platform
```

It uses:

- Node.js
- Express.js
- HTML
- CSS
- JavaScript
- Favoriot REST API

The dashboard can provide:

- Robot connection status
- Manual colour commands
- Automatic sorting
- Status checking
- Command history
- Activity logs
- Server-health information

---

## Web Platform Configuration

### 1. Copy the Environment Template

Inside:

```text
User Interface Part/web_platform
```

copy:

```text
.env.example
```

and rename the copy to:

```text
.env
```

### 2. Add Your Own Values

```env
PORT=3000

FAVORIOT_BASE_URL=https://apiv2.favoriot.com/v2

FAVORIOT_API_KEY=YOUR_FAVORIOT_API_KEY
FAVORIOT_TOKEN=YOUR_FAVORIOT_TOKEN

FAVORIOT_DEVICE_ID=YOUR_DEVICE_ID
FAVORIOT_DEVICE_DEVELOPER_ID=YOUR_DEVICE_DEVELOPER_ID

FAVORIOT_TOKEN_HEADER=apikey
```

Never upload the real `.env` file.

### 3. Install Dependencies

Open a terminal inside the `web_platform` folder:

```bash
npm install
```

### 4. Start the Server

```bash
npm start
```

If the project uses another script, run:

```bash
node server.js
```

### 5. Open the Dashboard

Open:

```text
http://localhost:3000
```

---

## Software and Libraries

The robotic arm project uses:

- C++
- Arduino Framework
- PlatformIO
- ESP32 Wi-Fi library
- WiFiClientSecure
- HTTPClient
- ArduinoJson
- Adafruit PWM Servo Driver library
- HuskyLens library
- Node.js
- Express.js
- HTML
- CSS
- JavaScript
- Favoriot REST API

---

## Project Structure

```text
robotarm/
├── include/
│   ├── secrets.h
│   └── secrets.example.h
│
├── lib/
│   └── Local libraries
│
├── src/
│   └── main.cpp
│
├── test/
│   └── Test files
│
├── User Interface Part/
│   └── web_platform/
│       ├── public/
│       ├── server.js
│       ├── package.json
│       ├── package-lock.json
│       ├── .env
│       └── .env.example
│
├── .gitignore
├── platformio.ini
└── README.md
```

---

## Firmware Setup

### 1. Open the Complete Project

Open the `robotarm` folder in Visual Studio Code.

Do not open only `main.cpp`.

### 2. Install PlatformIO

Install the PlatformIO extension in Visual Studio Code.

### 3. Create `secrets.h`

Copy `secrets.example.h` and rename it to:

```text
secrets.h
```

Enter your own Wi-Fi and Favoriot information.

### 4. Connect the ESP32

Connect the ESP32 using a USB cable.

### 5. Build the Project

Click:

```text
✓ Build
```

### 6. Upload the Firmware

Click:

```text
→ Upload
```

### 7. Open the Serial Monitor

Use the baud rate configured in `platformio.ini`.

### 8. Test Safety Functions

Before running the complete sequence:

1. Test individual servo controls
2. Test the home command
3. Test the emergency stop
4. Check the camera-protection zone
5. Test the gripper
6. Confirm HuskyLens detection
7. Run the sequence at a safe speed

---

## Troubleshooting

### Arduino or Library Files Cannot Be Found

Open the full `robotarm` folder and run:

```text
PlatformIO: Rebuild IntelliSense Index
```

### HuskyLens Is Not Detected

Check:

- HuskyLens communication mode
- UART TX and RX wiring
- Common ground
- Baud rate
- Power supply
- Correct recognition mode

### Servo Movement Is Unstable

Check:

- External servo power
- Shared ground
- PWM frequency
- Servo angle limits
- Sequence speed
- Mechanical obstruction

### Web Platform Cannot Start

Check:

- Node.js is installed
- `npm install` has completed
- `.env` exists
- Port 3000 is available
- Favoriot credentials are correct

### Favoriot Returns an Error

Check:

- API key
- Device developer ID
- Internet connection
- Request URL
- Authorization header
- Device permissions

---

## Limitations

The current prototype has several limitations:

- Pick-and-place positions are mainly pre-recorded
- Object location must remain within the expected area
- Wi-Fi is required for cloud communication
- Servo accuracy depends on calibration
- The gripper does not measure gripping force
- HuskyLens performance depends on camera position
- A failed sequence may require manual recovery
- The prototype is designed for a controlled environment

---

## Future Improvements

Possible improvements include:

- Dynamic object-position detection
- Camera-based grasp-point calculation
- Gripper force sensing
- Automatic sequence recovery
- Collision detection
- Closed-loop servo feedback
- More reliable robot-to-robot communication
- Tray handling
- Real-time video monitoring
- Additional object categories
- ROS integration
- Vision-language task commands

---

## Safety Notice

- Keep hands away from the arm during movement.
- Use an external power supply suitable for the servos.
- Connect all grounds correctly.
- Test at low speed first.
- Do not exceed servo limits.
- Keep the HuskyLens and cables outside the movement path.
- Always test the emergency-stop function.
- Disconnect power before making mechanical adjustments.

---

## Related Documentation

- [Main Project README](../README.md)
- [Mobile Robot Documentation](../Mobile_Robot/README.md)
