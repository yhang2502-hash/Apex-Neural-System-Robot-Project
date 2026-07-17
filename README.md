# Apex Neural System Robot Project

A collaborative robotics project combining a mobile robot and a robotic arm to complete an automated object transportation and sorting task.

The mobile robot is responsible for collecting and transporting coloured objects, while the robotic arm receives the task status and performs the final pick-and-place operation.

This project was developed during the ASEM AI and Robotics Programme.

---

## Project Demonstration

![Mobile robot and robotic arm collaboration setup](assets/robot-collaboration-setup.jpg)

The demonstration area contains:

- A starting area for the mobile robot
- A coloured-object collection area
- A predefined navigation path
- A dropping area near the robotic arm
- Blue, red, and yellow destination containers

---

## Project Scenario

The project demonstrates how two different robotic systems can communicate and work together.

The mobile robot starts from the marked starting area and navigates towards the collection area. It then transports a coloured object to the dropping area beside the robotic arm.

After reaching the dropping area, the mobile robot sends a completion status through the Favoriot IoT platform.

The robotic arm receives the status, identifies the required task, and automatically executes the corresponding pick-and-place sequence. The object is then placed into the correct coloured container.

After completing the task, the robotic arm returns to its home position and waits for the next object.

---

## Collaboration Sequence

```text
Mobile Robot Starts
        ↓
Moves to the Collection Area
        ↓
Collects and Transports the Object
        ↓
Arrives at the Dropping Area
        ↓
Sends Completion Status through Favoriot
        ↓
Robotic Arm Receives the Trigger
        ↓
HuskyLens Identifies the Required Task
        ↓
Robotic Arm Picks Up the Object
        ↓
Object Is Placed into the Correct Container
        ↓
Robotic Arm Returns to Home Position
```

---

## System Architecture

The project follows a simple:

```text
Sense → Decide → Act → Verify
```

| Stage | Mobile Robot | Robotic Arm |
|---|---|---|
| Sense | Detects route, line and obstacles | Detects tag, object or task trigger |
| Decide | Selects the movement sequence | Selects the pick-and-place sequence |
| Act | Transports the object | Picks and sorts the object |
| Verify | Confirms arrival at the dropping area | Confirms task completion and returns home |

---

## Main Components

### Mobile Robot

- Four-wheel mobile robot platform
- Arduino-compatible controller
- Line-tracking sensors
- Ultrasonic obstacle sensor
- DC motors and motor driver
- Object transportation platform
- Favoriot task-status communication

### Robotic Arm

- Six-degree-of-freedom robotic arm
- ESP32 controller
- PCA9685 servo driver
- HuskyLens AI vision sensor
- Servo motors and gripper
- Pre-recorded pick-and-place sequences
- Emergency-stop and home-position controls

### Communication and User Interface

- Favoriot IoT platform
- Wi-Fi communication
- Node.js and Express backend
- Web-based control dashboard
- Robot-status and task monitoring

---

## Detailed Documentation

More information about each robot is available here:

- [Mobile Robot Documentation](Mobile_Robot/README.md)
- [Robotic Arm Documentation](robotarm/README.md)

---

## Repository Structure

```text
Apex-Neural-System-Robot-Project/
├── Mobile_Robot/
│   ├── README.md
│   ├── src/
│   ├── include/
│   ├── lib/
│   └── platformio.ini
│
├── robotarm/
│   ├── README.md
│   ├── src/
│   ├── include/
│   ├── lib/
│   ├── User Interface Part/
│   └── platformio.ini
│
├── assets/
│   └── robot-collaboration-setup.jpg
│
├── docs/
│   └── Apex-Neural-System-Robot-Project-Presentation.pdf
│
├── .gitignore
└── README.md
```

---

## Presentation Slides

[View the project presentation slides](assets/Apex-Neural-System-Robot-Project-Presentation.pdf)

---

## Demonstration Video

The full collaboration video will be added here.

```text
Mobile robot navigation
→ Object transportation
→ Arrival-status communication
→ Automatic robotic-arm activation
→ Pick-and-place operation
```

[Watch the full robot collaboration demonstration](YOUR_VIDEO_LINK)

---

## Security Notice

Private credentials are not included in this repository.

Do not upload:

```text
.env
secrets.h
Wi-Fi passwords
Favoriot API keys
Access tokens
```

Users should create their own `.env` and `secrets.h` files using the provided example files.

---

## Acknowledgements

Special thanks to the Advanced Semiconductor Academy of Malaysia and the programme trainers for providing the robotics training, equipment, technical guidance, and opportunity to develop this collaborative robotics project.
