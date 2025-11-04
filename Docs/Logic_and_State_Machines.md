# ESP32 Line-Following Robot - Logic and State Machines

## Overview

This document describes the simplified logic and state machine implementation extracted from main.cpp of the ESP32 line-following robot.

## Pseudo-Code Logic

The main control loop follows this simplified logic:

```
1. Read line sensors
2. Based on current state:
   - Apply appropriate motor/servo control
   - Check for state transition conditions
   - Update to new state if conditions are met
3. Repeat
```

## State Machine

The robot transitions between the following states:

### States Definition
```
enum State {
  INIT,           // Initial state - system startup
  IDLE,           // Idle state - waiting for start command
  LINE_FOLLOW,    // Main line following operation
  AVOID_PREPARE,  // Prepare to avoid obstacle
  AVOID_PATH,     // Execute obstacle avoidance
  MERGE_SEARCH,   // Search to merge back to line after obstacle
  TURN_LEFT_PREPARE, // Prepare for left turn at junction
  SLOW_DOWN,      // Slow down when approaching goal
  STOP,           // Stop when goal is reached
  LOST_LINE       // Handle when line is lost
};
```

### State Transitions

```
INIT → LINE_FOLLOW (when button pressed)

LINE_FOLLOW → AVOID_PREPARE (when obstacle detected)
LINE_FOLLOW → TURN_LEFT_PREPARE (when junction detected and not after junction)
LINE_FOLLOW → SLOW_DOWN (when after junction and near goal)
LINE_FOLLOW → LOST_LINE (when line is lost)

AVOID_PREPARE → AVOID_PATH (after 500ms delay)
AVOID_PATH → MERGE_SEARCH (when obstacle cleared)
MERGE_SEARCH → LINE_FOLLOW (when line or junction detected)

TURN_LEFT_PREPARE → LINE_FOLLOW (after 800ms delay, mark junction passed)

SLOW_DOWN → STOP (when finish line detected)

LOST_LINE → remains in LOST_LINE (manual repositioning required)
```

### State Behaviors

#### INIT
```
- Stop motors
- Center servo (90°)
- Wait for start button press
- When button pressed, transition to LINE_FOLLOW
```

#### LINE_FOLLOW
```
- Compute error from line sensors
- Apply PID control to determine servo angle
- Apply soft-start logic for first run
- Check for obstacles, junctions, line loss
- Transition based on sensor input
```

#### AVOID_PREPARE
```
- Reduce motor speed to slow
- Steer servo to 130° (right turn)
- Wait 500ms before transitioning to AVOID_PATH
```

#### AVOID_PATH
```
- Normal motor speed
- Steer servo to 130° (right turn)
- Continue until obstacle is cleared
- Then transition to MERGE_SEARCH
```

#### MERGE_SEARCH
```
- Steer servo to 70° (left turn)
- Look for line or junction
- When found, return to LINE_FOLLOW
```

#### TURN_LEFT_PREPARE
```
- Reduce motor speed to slow
- Steer servo to 50° (left turn)
- Wait 800ms
- Mark junction as passed
- Reset encoder count
- Return to LINE_FOLLOW
```

#### SLOW_DOWN
```
- Reduce motor speed to slow
- Keep servo centered at 90°
- Wait for finish line detection
- Transition to STOP when finish detected
```

#### STOP
```
- Stop motors completely
- Center servo at 90°
- Infinite loop (program ends)
```

#### LOST_LINE
```
- Stop motors
- Center servo at 90°
- Print error message
- Remain stopped until manually repositioned
```

## Key Control Functions

- `computeError()` - Calculate deviation from line center
- `PID(e)` - Apply PID control to error value
- `setMotor(speed)` - Set motor speed
- `setServoAngle(angle)` - Set servo angle
- Sensor detection functions - Check for obstacles, junctions, etc.