# Pixhawk MAVLink Commands (v16)

This project uses explicit MAVLink commands to perform obstacle-avoidance maneuvers in GUIDED mode and then return to the AUTO mission.

## 1) SET_MODE → GUIDED
- Message: `SET_MODE` (command id 11 via `set_mode_send`).
- Flags: `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`.
- Payload example (ArduPilot/ArduRover):
  - `target_system`: autopilot system id
  - `custom_mode`: mode mapping value for `GUIDED`
- Usage in code: `MavlinkComm.ensure_guided()` calls `set_mode("GUIDED")` before sending position/attitude targets.

## 2) SET_ATTITUDE_TARGET (yaw change)
- Message: `SET_ATTITUDE_TARGET` (id 83).
- Fields we set:
  - `time_boot_ms`: monotonic ms
  - `target_system` / `target_component`
  - `type_mask`: ignore roll/pitch/body rates and throttle (yaw-only quaternion)
  - `q`: quaternion from Euler `(roll=0, pitch=0, yaw=target_yaw_rad)`
  - `body_roll_rate`, `body_pitch_rate`, `body_yaw_rate`: `0`
  - `thrust`: `0.5` (neutral)
- Example payload (conceptual):
  - `type_mask`: `ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE | ..._YAW_RATE_IGNORE`
  - `q`: `[cos(yaw/2), 0, 0, sin(yaw/2)]` for yaw-only target
- Usage in code: `MavlinkComm.yaw_relative()` → `_send_attitude_target(yaw_rad)`.

## 3) SET_POSITION_TARGET_LOCAL_NED
Message id: 84. Used in two ways:

### a) Yaw-only setpoint (BODY_NED frame)
- Frame: `MAV_FRAME_BODY_NED`
- Type mask: ignore X/Y/Z position, velocities, accelerations; keep `yaw` (optionally `yaw_rate`).
- Fields:
  - `x,y,z`: `0`
  - `vx,vy,vz`: `0`
  - `afx,afy,afz`: `0`
  - `yaw`: target yaw (radians, absolute)
  - `yaw_rate`: set or ignored
- Usage: `_send_yaw_setpoint(yaw_rad, yaw_rate)` inside `yaw_relative()`.

### b) Velocity-based motion forward/backward (BODY_NED frame)
- Frame: `MAV_FRAME_BODY_NED` (vehicle-forward X).
- Type mask: ignore position + acceleration, ignore yaw + yaw_rate (velocity-only).
- Fields:
  - `x,y,z`: `0`
  - `vx`: forward velocity (m/s, positive forward, negative backward)
  - `vy`: `0`
  - `vz`: `0`
  - `afx,afy,afz`: `0`
  - `yaw`, `yaw_rate`: ignored
- Example from code (`move_body`):
  - Mask: `X/Y/Z_IGNORE | AX/AY/AZ_IGNORE | YAW/YAW_RATE_IGNORE`
  - Sent at ~5 Hz for duration = `distance / speed`, then a zero-velocity message to stop.

### c) Combined yaw + velocity (not currently combined)
- The helpers keep yaw control separate (attitude + yaw setpoint) to simplify rover behavior.

## 4) SET_MODE → Original Mode
- After the maneuver clears the yellow zone, control returns to the mode that was active when the program started (or before switching to GUIDED):
  - `return_to_auto()` now restores the stored initial/pre-avoidance mode (fallback `AUTO`).
- Heartbeats then reflect `current_mode = <original>`, and avoidance idles until the next obstacle.

## Reference Sequence (used by AvoidanceController)
1. `SET_MODE GUIDED`
2. `SET_ATTITUDE_TARGET` (yaw to heading ± turn angle)
3. `SET_POSITION_TARGET_LOCAL_NED` (velocity forward `distance_m`)
4. `SET_ATTITUDE_TARGET` (yaw back to original heading)
5. `SET_POSITION_TARGET_LOCAL_NED` (velocity forward `distance_m`)
6. If clear: `SET_MODE AUTO`; else retry (up to 3 attempts, then stop).

