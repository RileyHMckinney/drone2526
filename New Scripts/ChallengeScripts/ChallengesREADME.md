# README – Operation Touchdown Autonomous Challenges
### UAV–UGV Coordination System – Challenge 1, Challenge 2, Challenge 3

This folder contains all software modules used for Challenges 1, 2, and 3 of Operation Touchdown.  
The system uses:

- A UAV (Pixhawk + Jetson + downward-facing camera)
- A UGV (Raspberry Pi + optional ESP32 + Akida BrainChip for Challenge 3)
- ArUco markers placed on the UGV and the goal
- A unified UAV → UGV communication system (TCP or ESP32 UART)
- Optional Akida obstacle avoidance

Each challenge builds on the previous one and uses a shared architecture with modular AV (aerial) and GV (ground) control scripts.

---

## Folder Structure

```
ChallengeScripts/
│
├── Challenge1/
│   ├── challenge1AV.py
│   └── challenge1GV.py
│
├── Challenge2/
│   ├── challenge2AV.py
│   └── challenge2GV.py
│
├── Challenge3/
│   ├── challenge3AV.py
│   └── challenge3GV.py
│
└── ChallengesREADME.md
```

---

## Challenge 1 – Autonomous Landing on a Moving UGV

### Goal
UAV must autonomously take off, track the UGV’s ArUco marker, keep it centered, and land on the UGV while the UGV drives in a straight line.

### Key Components

#### UAV:
- Tracks UGV marker ID = 1  
- Sends a “start” command to the UGV  
- Uses visual-servo centering logic  
- Performs slow, controlled descent until touchdown is detected  

#### UGV:
- Receives a start command  
- Drives forward at a fixed speed  
- Stops once UAV has landed  

### Scripts
- `challenge1AV.py`  
- `challenge1GV.py`

---

## Challenge 2 – UAV-Guided Vector Navigation (No Obstacle Avoidance)

### Goal
The UAV must guide the UGV to the goal marker by sending continuous 3D vectors computed from ArUco marker geometry.

### UAV Behavior (`challenge2AV.py`)
1. Takes off  
2. Tracks UGV marker (ID 1)  
3. Searches for goal marker (ID 0)  
4. Computes UGV → Goal 3D vector  
5. Streams vectors to the UGV via TCP or UART  
6. Descends as UGV approaches the goal  
7. Switches to landing mode once distance < 1 m  
8. Lands on the UGV  

### UGV Behavior (`challenge2GV.py`)
- Supports two comm modes:  
  - Mode 1: ESP32 UART  
  - Mode 2: Direct TCP  
- Interprets vector components:  
  - `x` → forward  
  - `y` → turn  
- No obstacle avoidance  
- Stops when near the goal  

### Scripts
- `challenge2AV.py`  
- `challenge2GV.py`

---

## Challenge 3 – BrainChip Akida Obstacle Avoidance + Vector Navigation

### Goal
Same vector navigation as Challenge 2, but with Akida-powered obstacle avoidance added.

### UAV Behavior (`challenge3AV.py`)
Identical to Challenge 2.  
No change required for Challenge 3.

### UGV Behavior (`challenge3GV.py`)
- Runs `AkidaObstacleDetector`  
- Uses `ObstacleAvoidance` to classify obstacles as LEFT, CENTER, RIGHT, or STOP  
- Overrides movement for obstacle avoidance:  
  - LEFT obstacle → turn RIGHT  
  - RIGHT obstacle → turn LEFT  
  - CENTER → STOP  
- Returns to vector-following mode once clear  
- Stops when near the goal  

### Scripts
- `challenge3AV.py`  
- `challenge3GV.py`

---

## Communication Modes (Challenges 2 and 3)

Both modes supported:

### Mode 1 — ESP32 UART
UAV → WiFi → ESP32 → UART → Raspberry Pi

### Mode 2 — Direct TCP (Recommended)
UAV Jetson → socket → Raspberry Pi

### User selects at runtime:
```
1 → UART
2 → TCP
```

---

## ArUco Marker Configuration

- UGV marker ID: **1**  
- Goal marker ID: **0**  
- Marker size: **0.254 m (10-inch square)**  
- Calibration file: `config/calibration_chessboard.yaml`

---

## Touchdown Detection Logic

Landing is confirmed only when all conditions are met:

- Altitude < 0.25 m  
- Vertical speed near zero  
- Marker area large  
- IMU Z acceleration variance low  

---

## Recommended Testing Sequence

### Challenge 1
1. Run `challenge1GV.py`  
2. Run `challenge1AV.py`

### Challenge 2
1. Run `challenge2GV.py`  
2. Run `challenge2AV.py`

### Challenge 3
1. Run `challenge3GV.py`  
2. Run `challenge3AV.py`

---

## Motor Driver Integration (Future Work)

Current GV scripts use placeholder motor functions:

```
motors_forward(speed)
motors_turn_left(speed)
motors_turn_right(speed)
motors_stop()
```

Replace these functions with your actual motor driver implementation (PWM, H-bridge, ESC, Roboclaw, etc).

---

## Conclusion

This folder provides a complete, modular, and competition-ready solution for:

- Challenge 1: Autonomous landing  
- Challenge 2: Vector navigation  
- Challenge 3: Vector navigation with Akida obstacle avoidance  

All scripts are structured for clarity, safety, and maintainability.
