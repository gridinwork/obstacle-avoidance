# Time-Based Movement System (Version 75)

## Overview

Version 75 replaces all angle-based turning and distance-based forward movement with **time-based turning and time-based forward movement**. GPS and compass are NOT used for obstacle avoidance maneuvers.

## Key Changes

### 1. Configuration Constants (`config.py`)

Added timing constants:
- `ROTATION_TIME_SEC = 1.5` - Duration for timed turns (left/right)
- `FORWARD_TIME_SEC = 0.8` - Duration for timed forward movement after turns
- `SPEED_CAP_PERCENT = 25.0` - Speed percentage cap for obstacle avoidance (maps to RC3 = 1250)

### 2. Movement Controller Updates (`movement_mavlink.py`)

#### Updated Steering Values
- **LEFT**: RC1 = 1000 (was 1100)
- **RIGHT**: RC1 = 2000 (was 1900)
- **CENTER**: RC1 = 1500 (unchanged)

#### New Timed Movement Functions

**`timed_turn_left(duration_seconds, speed_percent)`**
- Continuously sends ENABLE ON (RC5=1100)
- Sets REVERSE OFF (RC6=1100)
- Sends TURN LEFT (RC1=1000)
- Sets throttle according to speed_percent
- Keeps turning for `duration_seconds`
- Then STOPs: RC3=1000, RC1=1500, ENABLE OFF (RC5=1900)

**`timed_turn_right(duration_seconds, speed_percent)`**
- Same as left, but sends TURN RIGHT (RC1=2000)

**`forward_for(duration_seconds, speed_percent)`**
- Continuously sends ENABLE ON
- Sets REVERSE OFF
- Sets throttle mapped from speed_percent
- Continues for `duration_seconds`
- Then STOPs

#### Updated STOP Behavior
- RC3 = 1000 (throttle zero)
- RC1 = 1500 (center steering)
- Sends ENABLE OFF (RC5 = 1900)
- Disables any ongoing ENABLE loop thread

### 3. Obstacle Avoidance Updates (`avoidance.py`)

#### Yellow Zone Behavior (Time-Based)

**If obstacle is on the left side:**
```
1. Log: "YELLOW ZONE — Turning RIGHT by timer"
2. Execute: timed_turn_right(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)
3. Execute: forward_for(FORWARD_TIME_SEC, SPEED_CAP_PERCENT)
4. Execute: stop()
```

**If obstacle is on the right side:**
```
1. Log: "YELLOW ZONE — Turning LEFT by timer"
2. Execute: timed_turn_left(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)
3. Execute: forward_for(FORWARD_TIME_SEC, SPEED_CAP_PERCENT)
4. Execute: stop()
```

#### Removed Angle-Based Logic
- No angle calculations for turning
- No `turn_angle` computations
- No `angle_to_left_edge` / `angle_to_right_edge`
- No compass-based turning
- No GPS-based distance calculations for movement

**Note:** Math library is still used for LiDAR point coordinate conversion (`_to_xy` method), which is necessary for obstacle detection, not for turning angles.

### 4. Logging

Added clear debug logs:
- "Timed turn LEFT started (X sec)"
- "Timed turn RIGHT started (X sec)"
- "Timed turn finished"
- "Driving forward for X seconds"
- "STOP — ENABLE OFF"

## RC Channel Mapping

| Channel | Function | Values |
|---------|----------|--------|
| RC1 | Steering | 1000=LEFT, 1500=CENTER, 2000=RIGHT |
| RC3 | Throttle | 1000-2000 (formula: 1000 + speed_percent × 10) |
| RC5 | ENABLE | 1100=ON, 1900=OFF |
| RC6 | REVERSE | 1100=FORWARD, 1900=BACKWARD |
| RC7 | ARMING | 1900=ARM, 1100=DISARM |

## ENABLE Signal Behavior

- **ENABLE ON** (RC5=1100) must be sent continuously every 200ms while robot is moving or turning
- **ENABLE OFF** (RC5=1900) is sent when STOP is called
- The ENABLE loop thread automatically handles continuous sending during movement

## Testing

To test the timed movement system:

```python
from movement_mavlink import MovementController
from config import ROTATION_TIME_SEC, FORWARD_TIME_SEC, SPEED_CAP_PERCENT

controller = MovementController()
controller.connect("/dev/serial0", 57600)
controller.arm()

# Test timed turn
controller.timed_turn_left(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)

# Test timed forward
controller.forward_for(FORWARD_TIME_SEC, SPEED_CAP_PERCENT)

# Stop
controller.stop()
controller.disarm()
```

## Migration Notes

- All yellow zone avoidance now uses timed turns and timed forward movement
- No GPS or compass data is required for obstacle avoidance maneuvers
- Direction selection (left/right) is still based on LiDAR obstacle counts, but the actual turn is time-based, not angle-based
- Manual control and other modes remain unchanged
