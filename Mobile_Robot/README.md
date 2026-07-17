# Mobile Robot

The mobile robot is the transportation unit of the **Apex Neural System Robot Project**.

Its main role is to move from the starting area, navigate through the collection area, transport an object, and stop at the robotic-arm dropping station. After completing the delivery, it sends a completion status so that the robotic arm can begin its pick-and-place operation.

[← Back to Main Project](../README.md)

---

## Overview

The mobile robot uses a four-wheel Mecanum platform that can move in multiple directions.

It performs a predefined movement sequence while using line-tracking and ultrasonic sensors to support navigation and obstacle safety.

The robot works together with the robotic arm through the following process:

```text
Start Position
      ↓
Navigate to Collection Area
      ↓
Collect or Carry Object
      ↓
Transport Object
      ↓
Reach Dropping Area
      ↓
Send Completion Status
      ↓
Robotic Arm Starts
```

---

## Main Responsibilities

The mobile robot is responsible for:

1. Starting from the marked starting position
2. Moving through the predefined route
3. Detecting the navigation line
4. Checking for obstacles
5. Carrying the selected object
6. Reaching the dropping area beside the robotic arm
7. Sending a task-completion status
8. Remaining stationary while the robotic arm completes the next task

---

## Demonstration Scenario

The robot begins at the marked `START` position.

It moves towards the collection area containing coloured objects. After obtaining or carrying an object, the robot follows the programmed path towards the dropping area.

When the robot reaches the robotic-arm station, it stops and sends a completion message through the communication system.

The robotic arm reads this status and begins the correct object-sorting sequence.

---

## Hardware

The mobile robot prototype uses:

- Keyestudio KS0560 four-wheel Mecanum robot platform
- Arduino-compatible controller
- Four DC motors
- Motor-driver board
- Four Mecanum wheels
- Three line-tracking sensors
- Ultrasonic distance sensor
- Infrared remote receiver
- Object-carrying platform
- Rechargeable battery supply
- ESP32 or communication controller for cloud integration

---

## Sensor Functions

### Line-Tracking Sensors

The line-tracking sensors are used to detect the marked path underneath the robot.

The three sensors allow the program to determine whether the robot should:

- Continue moving forward
- Adjust towards the left
- Adjust towards the right
- Stop or change direction

The exact reading depends on the surface and sensor configuration. In the prototype, the sensors are calibrated according to the black navigation line.

### Ultrasonic Sensor

The ultrasonic sensor measures the distance between the robot and an object in front of it.

It is used for:

- Obstacle detection
- Collision prevention
- Safe stopping
- Short-range distance checking

When an obstacle is detected within the configured safety distance, the robot can stop and wait until the route is clear.

### Infrared Receiver

The infrared receiver supports manual testing and movement control using a remote controller.

It is useful during:

- Motor testing
- Movement calibration
- Route testing
- Emergency manual control

---

## Mecanum Movement

The Mecanum wheel system allows the robot to perform:

- Forward movement
- Backward movement
- Left movement
- Right movement
- Diagonal movement
- Clockwise rotation
- Anticlockwise rotation
- Complete stop

Motor-speed calibration is required because each motor may rotate at a slightly different speed.

---

## Operating Sequence

### Step 1 — System Start

The controller initialises:

- Motors
- Line sensors
- Ultrasonic sensor
- Infrared receiver
- Communication system

The robot remains stopped until the movement sequence starts.

### Step 2 — Leave the Starting Area

The mobile robot moves forward from the marked start position.

The motor speed is adjusted to keep the robot moving in the intended direction.

### Step 3 — Navigate through the Route

The robot follows the predefined movement and line-tracking logic.

Sensor readings are used to correct the robot when it moves away from the marked route.

### Step 4 — Check for Obstacles

The ultrasonic sensor continuously checks the distance in front of the robot.

If an obstacle is too close:

```text
Obstacle Detected
       ↓
Stop Motors
       ↓
Wait for Clear Route
       ↓
Continue Movement
```

### Step 5 — Transport the Object

The robot carries the selected object from the collection area towards the dropping area.

### Step 6 — Stop at the Robotic-Arm Station

The robot stops at the designated dropping position near the robotic arm.

Correct positioning is important because the robotic arm uses a pre-recorded pick-and-place sequence.

### Step 7 — Send Completion Status

The mobile robot sends a completion status through Favoriot or the configured communication channel.

Possible status values include:

```text
DONE
FINISH
FINISHED
COMPLETE
COMPLETED
TaskMiddle_complete
TaskLeft_complete
TaskRight_complete
```

The robotic-arm controller checks these messages before starting its sequence.

---

## Communication with the Robotic Arm

The mobile robot and robotic arm do not need to be directly connected by cable.

The collaboration can use:

- Favoriot cloud communication
- Wi-Fi communication
- Serial communication
- REST API requests
- Device-status messages

The basic communication flow is:

```text
Mobile Robot Completes Delivery
             ↓
Completion Status Sent
             ↓
Favoriot Device Stream Updated
             ↓
Robotic Arm Reads Status
             ↓
Pick-and-Place Sequence Starts
```

---

## Software

The mobile robot project uses:

- C++
- Arduino Framework
- PlatformIO
- Visual Studio Code
- Motor-control libraries
- Ultrasonic distance measurement
- Line-tracking logic
- Infrared remote control
- Wi-Fi or Favoriot communication

---

## Project Structure

```text
Mobile_Robot/
├── include/
│   └── Project header files
├── lib/
│   └── Local libraries
├── src/
│   └── main.cpp
├── test/
│   └── Test files
├── .gitignore
├── platformio.ini
└── README.md
```

---

## Setup Instructions

### 1. Install PlatformIO

Install:

- Visual Studio Code
- PlatformIO extension

### 2. Open the Complete Project Folder

Open:

```text
Mobile_Robot
```

Do not open only `main.cpp`, because PlatformIO needs the complete project folder and `platformio.ini`.

### 3. Connect the Controller

Connect the mobile-robot controller to the computer using a USB cable.

### 4. Select the Correct Board and Port

Check the environment inside:

```text
platformio.ini
```

Then select the correct serial port.

### 5. Build the Project

Use the PlatformIO build button:

```text
✓ Build
```

### 6. Upload the Firmware

Use:

```text
→ Upload
```

### 7. Test the Robot

Place the robot on the demonstration field and confirm:

- Motor directions are correct
- Line sensors detect the route
- Ultrasonic readings are stable
- The robot stops at the intended destination
- The completion status is sent correctly

---

## Calibration

Before running the complete sequence, check:

### Motor Calibration

Because motor speeds may differ, adjust the speed value for each wheel until the robot moves straight.

### Line Sensor Calibration

Test the sensor values on:

- Black navigation line
- White background

Update the program logic according to the measured values.

### Ultrasonic Calibration

Confirm that the measured distance is reasonable and stable.

Use a safe stopping distance appropriate for the robot speed.

---

## Safety Notes

- Place the robot on the floor before starting.
- Keep cables away from the wheels.
- Do not place hands near moving wheels.
- Test at low speed first.
- Keep a clear emergency-stop method available.
- Stop the robot if sensor readings become unstable.
- Make sure the dropping area is correctly aligned with the robotic arm.

---

## Limitations

The current prototype has several limitations:

- Navigation mainly depends on a controlled test environment
- Predefined movement may require recalibration
- Wheel slipping can affect positioning
- Ultrasonic sensors may be less reliable on angled or soft surfaces
- Line tracking depends on lighting and surface contrast
- Precise docking is required for the robotic arm
- Cloud communication depends on Wi-Fi availability

---

## Future Improvements

Possible improvements include:

- LiDAR-based navigation
- SLAM mapping and localisation
- Wheel encoders
- IMU direction correction
- Dynamic obstacle avoidance
- Automatic docking
- Camera-based object detection
- Real-time location tracking
- Battery monitoring
- Task-failure recovery
- Direct robot-to-robot communication

---

## Related Documentation

- [Main Project README](../README.md)
- [Robotic Arm Documentation](../robotarm/README.md)
