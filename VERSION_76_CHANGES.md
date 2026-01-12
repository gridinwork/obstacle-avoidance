# Version 76 - Full Cleanup Summary

## Overview

Version 76 is a complete cleanup that removes ALL old movement logic and uses ONLY RC override commands for obstacle avoidance. No GPS, compass, or angle-based calculations are used.

## Key Changes

### 1. Movement Controller (`movement_mavlink.py`)

**Removed:**
- `forward()` - old continuous forward function
- `backward()` - old continuous backward function  
- `left()` - old instant turn function
- `right()` - old instant turn function
- `execute_json_command()` - old JSON command handler
- All references to angle-based or GPS-based movement

**Kept Only:**
- `timed_turn_left(duration, speed_percent)` - timed left turn
- `timed_turn_right(duration, speed_percent)` - timed right turn
- `forward_for(duration, speed_percent)` - timed forward movement
- `backward_for(duration, speed_percent)` - timed backward movement (NEW)
- `stop()` - stop with ENABLE OFF
- `arm()` / `disarm()` - system control

**Fixed:**
- REVERSE ON = RC6 = 2000 (was 1900)
- REVERSE OFF = RC6 = 1100
- STOP properly disables ENABLE loop thread

### 2. Obstacle Avoidance (`avoidance.py`)

**Removed:**
- All angle-based turning calculations
- GPS-based distance calculations
- `MavlinkMotionCommands` usage
- `_apply_speed()` function
- `_drive_until_clear()` function
- `_forward_until_clear()` function
- Old `_reverse_until_yellow()` logic

**New Behavior:**
- **Yellow Zone**: 
  - Pauses LIDAR scanning during maneuver
  - Executes `timed_turn_left/right()` based on obstacle side
  - Executes `forward_for()` after turn
  - Resumes LIDAR scanning after maneuver completes
  
- **Red Zone**:
  - Pauses LIDAR scanning
  - Executes `backward_for()` to reverse
  - Resumes LIDAR scanning after reverse

**LIDAR Management:**
- Sets `action_in_progress = True` before maneuvers
- Clears LIDAR points during maneuvers
- Resumes processing after maneuvers complete
- Prevents "looping stops" by freezing obstacle detection during movement

### 3. Motion Control (`motion_control.py`)

**Removed:**
- `MavlinkMotionCommands` import and usage
- `_update_motion_commands()` function
- `apply_speed()` function
- `move_backward_one_meter()` function
- All fallback code to old system

**Kept:**
- `stop_vehicle()` - uses MovementController.stop()
- `handle_key()` - uses timed functions for manual control
- `enable_manual()` / `disable_manual()` - manual mode control

### 4. RC Channel Mapping (Final)

| Channel | Function | Values |
|---------|----------|--------|
| RC1 | Steering | 1000=LEFT, 1500=CENTER, 2000=RIGHT |
| RC3 | Throttle | 1000-2000 (formula: 1000 + speed_percent × 10) |
| RC5 | ENABLE | 1100=ON, 1900=OFF |
| RC6 | REVERSE | 1100=FORWARD, 2000=BACKWARD |
| RC7 | ARMING | 1900=ARM, 1100=DISARM |

### 5. ENABLE Signal Behavior

- ENABLE ON (RC5=1100) sent continuously every 200ms during movement
- ENABLE loop thread automatically manages this
- STOP disables ENABLE loop and sends ENABLE OFF (RC5=1900)

### 6. Yellow Zone Maneuver Flow

```
1. Detect obstacle in yellow zone
2. Set action_in_progress = True
3. Clear LIDAR points (freeze scanning)
4. Execute timed_turn_left/right(ROTATION_TIME_SEC, SPEED_CAP_PERCENT)
5. Execute forward_for(FORWARD_TIME_SEC, SPEED_CAP_PERCENT)
6. Execute stop()
7. Set action_in_progress = False (resume LIDAR scanning)
8. Restore previous mode
```

### 7. Code Cleanup

- Removed all unused imports
- Removed commented legacy code
- Removed dead functions
- Updated all docstrings
- Version updated to "76"

## Testing Checklist

- [ ] Yellow zone obstacle avoidance uses timed turns
- [ ] LIDAR scanning pauses during maneuvers
- [ ] LIDAR scanning resumes after maneuvers
- [ ] No "looping stops" occur
- [ ] STOP properly disables ENABLE
- [ ] REVERSE uses RC6=2000 for ON
- [ ] Manual control works with timed functions
- [ ] No angle/GPS/compass calculations in movement code

## Files Modified

- `movement_mavlink.py` - Complete rewrite, only timed functions
- `avoidance.py` - Removed old logic, added LIDAR pause/resume
- `motion_control.py` - Removed MavlinkMotionCommands, simplified
- `config.py` - Timing constants (unchanged from v75)
- `VERSION` / `version.py` - Updated to "76"

## GitHub

Successfully pushed to: https://github.com/gridinwork/obstacle-avoidance.git
Commit: "Version 76 - Full cleanup, new timed-movement obstacle avoidance, removed all old logic"
