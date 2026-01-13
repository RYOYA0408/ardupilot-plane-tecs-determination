# Plane TECS Auto-Tune Script Manual

Lua Script for ArduPlane 4.6.x

Script name: `TecsTune_dev4_6.lua`  
This is a Lua script for the ArduPlane 4.6 series that **automatically determines the primary TECS parameters**.

The basic design concept and the meanings of the parameters are based on the following documents:  
- [TECS Tuning Guide (Official ArduPilot Documentation)](https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html)  
- [ArduPlane 4.6 Parameter List](https://ardupilot.org/plane/docs/parameters-Plane-beta-V4.6.0.html)

Script author: Ryoya Fukada

---

## **1. Script Overview**

`TecsTune_dev4_6.lua` periodically acquires in-flight vehicle states (e.g., airspeed, climb rate, acceleration, altitude, and throttle percentage) and stepwise determines TECS-related parameters based on a steady-state assessment using MAE.  
The procedure is divided into Phases 1–6 and is executed in synchronization with mission progression (e.g., WP/DO_JUMP/TAKEOFF) in the flight plan.

The following table summarizes each phase, its objective/content, and the parameters to be determined.

| Phase | Objective / Content                                   | Parameters to Determine                                                                 |
|------:|--------------------------------------------------------|-----------------------------------------------------------------------------------------|
| 1     | Deceleration test: search for a safe minimum speed near stall | `AIRSPEED_MIN`                                                                          |
| 2     | Acceleration test: search for a safe range on the high-speed side | `AIRSPEED_MAX`                                                                          |
| 3     | Acceleration preparation before climb                  | (No parameter determination: acceleration only)                                          |
| 4     | Determine maximum climb rate and climb attitude        | `THR_MAX`, `TECS_CLMB_MAX`, `TECS_PITCH_MAX`, `TECS_PITCH_MIN`, `TECS_SINK_MAX`          |
| 5     | Determine stable sink rate under a pseudo-glide condition | `TECS_SINK_MIN`                                                                         |
| 6     | Determine trim throttle under cruise conditions        | `TRIM_THROTTLE`                                                                         |

Within each phase, the script temporarily rewrites the following parameters to establish the required test conditions:

- `AIRSPEED_CRUISE`, `AIRSPEED_MIN`, `AIRSPEED_MAX`
- `TECS_SPDWEIGHT`, `TECS_PITCH_MAX`, `TECS_PITCH_MIN`
- `TECS_CLMB_MAX`, `TECS_SINK_MAX`, `TECS_SINK_MIN`
- `THR_MIN`, `THR_MAX`, `THR_SLEWRATE`, `TRIM_THROTTLE`
- `TKOFF_THR_*` series

Finally, the parameters are restored to safe values by `reset_all_param()`.

---

### 2.1 ArduPlane / Lua Environment

- Target: **ArduPlane 4.6.x**
- Script Placement
- SITL: `ArduPilot/scripts/`
- Actual Device: SD card `/APM/scripts/`
- Lua-Related Settings
- `SCR_ENABLE = 1` (Lua enabled)
- Other basic Lua-related settings must be complete

### 2.2 RC Switch Configuration

Script ON/OFF is controlled via an RC switch

- Assign `RCx_OPTION = 300 (Scripting3)` to any RC channel
- Referenced in the script as follows:

```lua
local scripting_rc = rc:find_channel_for_option(300)
```

Switch position handling (assuming 3-position switch):

- `sw_pos == 0`: OFF (Script stops, phase saved)
- `sw_pos == 1`: ON (TECS tuning executed)
- `sw_pos == 2`: Unused (reserved for future expansion)

### 2.3 Operational Flight Modes

- Scripts only execute in **AUTO mode (mode=10)**.

```lua
local mode = vehicle:get_mode()
if mode == 10 and scripting_rc then
...
end
```

---

## **3. Definition of Constants and Mission Numbers (Conforming to Current Implementation)**

### 3.1 Initial Values for Key Parameters (Laboratory Standard)

- `arspd_min_lab = 20` (Initial value for AIRSPEED_MIN)
- `arspd_max_lab = 30` (Initial value for AIRSPEED_MAX)
- `arspd_cruise_lab = 25` (AIRSPEED_CRUISE default)
- `trim_thr_lab = 65` (TRIM_THROTTLE default)
- `thr_slr_lab = 33` (THR_SLEWRATE default)

### 3.2 Parameters for TECS Tune

- `thr_min = 10` (THR_MIN: For Phases 1, 2, 5, 6)
- `thr_max = 100` (THR_MAX)
- `temp_thr_min = 10` (Temporary THR_MIN for Phase 4)
- `temp_thr_slr = 20` (Temporary THR_SLEWRATE for Phase 5)
- `ttc = 5.0` (TECS_TIME_CONST for Phase 5 (currently commented out))

### 3.3 Mission Number (Depends on Flight Plan)

- `default_circle_cmd = 3` (Default orbit at 80 m altitude)
- `default_circle_cmd2 = 2` (Initial orbit waypoint)
- `accel_sdby_cmd_num = 5` (Acceleration “standby” before ascent)
- `dojump_aft_cmd_num = 8` (Command reached after ascent)
- `tkoff_before_cmd = 6` (Command to return to when aborting/retrying ascent)
- `tkoff_cmd_num = 10` (TAKEOFF command used during Phase 4 ascent)
- `phase5_spdw_cmd = 12` (Phase 5 speed distribution switch trigger)
- `phase5_mes_cmd = 13` (Phase 5 measurement start signal command ※For design identification purposes)

---

## **4. Key Sections Users Should Modify**

### 4.1 Fixed Parameters at the Top of the Script

```lua
-- Retrieve/set required fixed variables
local k_throttle = 70 -- Throttle channel number
local gravity = 9.80665 -- Gravitational acceleration
-- Default parameters for laboratory reference
local arspd_min_lab = 20 -- Default minimum speed in the laboratory
local arspd_max_lab = 30 -- Default maximum speed in the lab
local arspd_cruise_lab = 25 -- Cruise speed in the lab
local trim_thr_lab = 65 -- Default trim throttle rate in the lab
local thr_slr_lab = 33 -- Throttle slew rate in the lab
-- TECS Tune parameter initial values
local arspd_cruise = 25 -- Target cruise speed
local set2up_arspd = 28 -- Post-acceleration speed during acceleration phase
local thr_min = 10 -- Minimum throttle rate for Phase 1, 2, 5, 6
local thr_max = 100 -- Maximum throttle rate
local temp_thr_min = 10 -- Temporary minimum throttle rate for Phase 4 (to allow flexible speed adjustment before ascent)
local temp_thr_slr = 20 -- Temporary throttle slew rate for Phase 5
```

| Category | Variable Name | Role / Meaning | Unit / Type | Example Initial Value | Modification Guide |
| --- | --- | --- | --- | --- | --- |
| Fixed Constant | `k_throttle` | `SERVOx_FUNCTION` number referenced by `SRV_Channels:get_output_scaled()` (throttle output source) | int | 70 | Match the `SERVOx_FUNCTION` value assigned to the throttle output of the aircraft in use. |
| Fixed Constant | `gravity` | Gravitational acceleration | m/s^2 (float) | 9.80665 | Do not change in principle (use standard gravitational acceleration). |
| Laboratory Standard (Initial Value) | `arspd_min_lab` | Initial `AIRSPEED_MIN` (provisional minimum speed) | m/s (float) | 20 | Set to known safe minimum speed (speed not leading to stall). |
| Laboratory Standard (Initial Value) | `arspd_max_lab` | Initial `AIRSPEED_MAX` (Provisional Upper Speed Limit) | m/s (float) | 30 | Set to known safe upper limit for high-speed range (within range satisfying airframe strength, controllability, and thrust margin). |
| Laboratory Standard (Initial Value) | `arspd_cruise_lab` | Initial `AIRSPEED_CRUISE` (Laboratory Standard Cruise Speed) | m/s (float) | 25 | Set to a representative cruise speed for normal operation. |
| Laboratory Standard (Initial Value) | `trim_thr_lab` | Initial `TRIM_THROTTLE` (Reference trim throttle rate during cruise) | % (int/float) | 65 | Set to the steady-state throttle rate during actual aircraft horizontal cruise (known value). |
| Laboratory Standard (Initial Value) | `thr_slr_lab` | Initial `THR_SLEWRATE` (throttle change rate limit) | %/s (float) | 33 | Set based on requirements for steepness of thrust response and suppression of throttle command fluctuations. |
| TECS Tune (Initial Value) | `arspd_cruise` | Cruise target speed referenced during TECS Tune execution | m/s (float) | 25 | Set to the normal operational cruise target speed (typically identical to `arspd_cruise_lab`). |
| TECS Tune (Initial Value) | `set2up_arspd` | Target speed to reach before climb initiation (acceleration phase target) | m/s (float) | 28 | Set to a speed ensuring stall margin at climb initiation (set higher than cruise speed). |
| TECS Tune (Default) | `thr_min` | Minimum throttle percentage (used in Phase 1, 2, 5, 6, etc.) | % (int/float) | 10 | Set to the lower limit where the propulsion system operates stably (within the range preventing misfires or shutdowns). |
| TECS Tune (Default) | `thr_max` | Maximum throttle rate (upper limit within the script) | % (int/float) | 100 | Set to a value consistent with transmitter settings or safety-related upper limits. |
| TECS Tune (Initial Value) | `temp_thr_min` | Temporary minimum throttle rate for Phase 4 (for speed control flexibility) | % (int/float) | 10 | Set based on Phase 4 speed maintenance requirements and the lower limit for stable propulsion system operation. |
| TECS Tune (Initial Value) | `temp_thr_slr` | Temporary `THR_SLEWRATE` for Phase 5 (change rate limit for command variation suppression) | %/s (float) | 20 | Set based on Phase 5 requirements for stability assessment (throttle variation suppression) and responsiveness. |

### 4.2 Update Period and Log Period

```lua
local FREQUENCY = 20 -- Code sampling frequency (ms) Changed 20251028
local dt = FREQUENCY / 1000 -- Step size during steady-state evaluation
local dt_log = 0.02 -- Sampling interval time for BIN log files
local obs_time = 4.0 -- Observation time when determining AIRSPEED_MIN/MAX
local obs_sink_time = 3.0 -- Observation time for steady glide phase
local obs_climb_time = 3.5 -- Observation time for climb phase
```

- Changing `FREQUENCY` automatically updates `dt`.
- Adjusting `dt_log` when changing the log cycle helps maintain sensitivity to the MAE threshold.

### 4.3 Steady-State Evaluation Thresholds (σ, MAE)

```lua
local arspd_sigma_log = 0.52
local slope_sigma_log = 0.55
local dh_sigma_log = 0.76
local height_sigma_log = 0.71

local margin_scale = 1.0
local arspd_mae = 0.53
local slope_mae = 0.55
local dh_mae = 0.61
local height_mae = 0.61
```

- Set based on values derived from experimental logs for specific days.
- Ideally recalculate from logs when the aircraft or environment changes.
- To loosen the criteria, set `margin_scale` to 1.2–1.5.

### 4.4 Speed Table (Phase1,2)

```lua
-- Phase1: Deceleration Direction (Search for AIRSPEED_MIN)
local decel_arspd_max = 24
local decel_arspd_min = 16

-- Phase2: Acceleration Direction (AIRSPEED_MAX Search)
local accel_arspd_min = 26
local accel_arspd_max = 35
```

- Phase1: Decelerate in 1 m/s increments from `decel_arspd_max` to `decel_arspd_min`.
- Phase2: Accelerate from `accel_arspd_min` to `accel_arspd_max` in 1 m/s increments.

Adjusted to match the aircraft's safe speed range

### 4.5 Climb Phase Settings (Phase4)

```lua
local target_alt = 140 -- Target altitude for TKOFF command [m]
local target_alt_limit = target_alt - 5.0 -- Altitude to terminate climb flight

local climb_cnt = 0 -- Initialize climb count
local climb_cnt_max = 5 -- Maximum climb count
local next_pitch_cmd= 23 -- Initial climb pitch command [deg]
```

- `target_alt`: Target altitude for the TKOFF command
- `target_alt_limit`: Altitude at which climb flight is terminated (Setting the TKOFF command value directly as the threshold causes it to enter Loiter mode unexpectedly)
- `climb_cnt_max`: Maximum number of climb retries -->> Forced transition to Phase 5 if exceeded
- `next_pitch_cmd`: Initial TAKEOFF pitch angle

### 4.6 Phase 5 (Pseudo-Glide) Altitude Conditions

```lua
local phase5_height_limit = 30
local phase5_cnt = 0
local phase5_cnt_max = 3
```

If altitude falls below `phase5_height_limit` during Phase 5, the flight will be interrupted once for safety and restart the climb.

### 4.7 Switching Sensor Acquisition Functions (Hardware/SITL)

`update_sensors()` has two versions: one for hardware and one for SITL+noise.

#### Hardware Version (without ``+ noise()``)

```lua
function update_sensors()
local pitch_deg = math.deg(ahrs:get_pitch())
local arspd_now = ahrs:airspeed_estimate()
local accel_x_now = (ahrs:get_accel():x() - gravity*math.sin(ahrs:get_pitch()))
local dh_now = - ahrs:get_velocity_NED():z()
local height_now = - ahrs:get_relative_position_NED_home():z()
local throttle_now= SRV_Channels:get_output_scaled(k_throttle)
return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
end
```

#### SITL + Add noise generated by the ``noise(sigma)`` function to ``noise()``

```lua
function noise(sigma)
local a, b
-- Ensure a > 0 to prevent z calculation overflow
repeat
a = math.random()
until a > 0
b = math.random()
local z = math.sqrt(-2*math.log(a)) * math.cos(2*math.pi*b)
return z * sigma
end
```

```lua
function update_sensors()
local pitch_deg = math.deg(ahrs:get_pitch())
local arspd_now = ahrs:airspeed_estimate() + noise(arspd_std)
local accel_x_now = (ahrs:get_accel():x() - gravity*math.sin(ahrs:get_pitch())) + noise (slope_std)
local dh_now = - ahrs:get_velocity_NED():z() + noise(dh_std)
local height_now = - ahrs:get_relative_position_NED_home():z() + noise(height_std)
local throttle_now= SRV_Channels:get_output_scaled (k_throttle)
return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
end
```

- **For actual operation**: Uncomment the actual hardware version and comment out the version with noise.
- **For SITL experiments**: Keep the version with noise to reproduce fluctuations similar to the logs.

---

## **5. Key Functions (Excerpt)**

### 5.1 `update()`

Starts/pauses/resumes the script based on the RC Option 300 switch state.
When paused, saves the Phase and trial count to `SCR_USER1~3` for restoration upon resumption.

### 5.2 `update_sensors()`

Acquires airspeed, climb rate, altitude, IMU acceleration, throttle rate, etc., and passes them to Phase determination/evaluation.

### 5.3 `make_stab_ctx()` / `check_stability()`

Generates a buffer (observation window) for stability evaluation and performs stability determination based on MAE.

### 5.4 `reset_all_param()` / `reset_all_flags()`

Resets parameters and flags to safe values during Phase transitions or interruptions.
Additionally, uses `SCR_USER6` as the **TECS Tune execution flag (ON/OFF)**.

### 5.5 Using `SCR_USER4` (Addendum)

Since `TECS_PITCH_MIN` is temporarily overwritten in Phase 5, the value obtained in Phase 4 is saved to `SCR_USER4` and restored to its original state upon return.

---

## **6. Description of Each Phase**

### 6.1 Phase 0: Standby/Initialization Processing

- Switch OFF: Do nothing (or perform interrupt handling)
- Switch ON: Reset to initial values via `param_cleanup()` and transition to Phase 1

---

### 6.2 Phase 1: Determining AIRSPEED_MIN (Deceleration Table)

1. Monitor current airspeed, etc., while tracking the target speed in the deceleration table
2. If steady-state determination succeeds in the observation window, proceed to the next deceleration step
3. Terminate under either condition:
- Failed to determine steady-state within the specified number of attempts (timeout)
- Minimum step reached
4. Save the resulting lower limit speed that avoids stall as `AIRSPEED_MIN`

---

### 6.3 Phase 2: Determining AIRSPEED_MAX (Acceleration Table)

Using a concept symmetrical to Phase 1, gradually increase speed using the acceleration table to perform steady-state determination and determine `AIRSPEED_MAX`.

---

### 6.4 Phase 2.5: Transition Hold Before Climb (Safety-Side Parameter Settings)

Before entering Phase 3 (pre-climb acceleration), temporarily set the following to create a “stall-resistant hold”:

- Set `AIRSPEED_CRUISE` to the pre-climb target
- `TECS_PITCH_MAX = 0`, `TECS_PITCH_MIN = 0` (to avoid extreme nose-up/down)
- Set TAKEOFF-related parameters (e.g., `TKOFF_*`)
- Transition to `mission:set_current_cmd(accel_sdby_cmd_num)` as appropriate to start Phase 3

---

### 6.5 Phase 3: Pre-climb Acceleration (Reaching Rotation Speed)

- Purpose: Ensure **sufficient rotation speed** to prevent stalling immediately after Phase 4 climb initiation
- Decision: Transition to Phase 4 upon reaching target speed `set2up_arspd`

---

### 6.6 Phase 4: Determining Maximum Climb Rate, Pitch Limit, and Maximum Descent Rate (Using TAKEOFF)

#### 6.6.1 Execution Overview

- Enter the **TAKEOFF command segment** via `mission:set_current_cmd(tkoff_cmd_num)` to execute the climb attempt
- During the climb attempt, determine parameters like `TECS_CLMB_MAX` based on steady-state criteria (climb rate, speed, etc.)
- If the attempt fails (e.g., stability not achieved or altitude condition reached), return to `tkoff_before_cmd` for retry

#### 6.6.2 TECS_PITCH_MIN Backup

To temporarily modify `TECS_PITCH_MIN` in Phase 5, save the `TECS_PITCH_MIN` determined in Phase 4 to `SCR_USER4`.

---

### 6.7 Phase 5: Determining TECS_SINK_MIN (Minimum Descent Rate)

#### 6.7.1 Flow

1. Set parameters before descent start (e.g., `TECS_PITCH_MIN = -30`, `TECS_SPDWEIGHT = 1`, etc.)
2. Set the speed distribution switch preparation flag `spdw_sw` triggered by reaching `phase5_spdw_cmd`
3. Upon reaching `accel_sdby_cmd_num`, set the Phase 5 measurement start flag `phase5_start_sw`
4. After measurement starts, to “simulate glide state”:
- `TECS_SPDWEIGHT = 2.0` (Speed Priority) 
- `TRIM_THROTTLE = thr_min` (Set throttle toward minimum)
are configured to perform steady-state determination in a state where **the influence of altitude error is weakened**.

#### 6.7.2 Notes on Current Implementation

The current code **effectively skips the acceleration steady-state check (always true)** during Phase 5 steady-state determination.
This is a temporary workaround for the issue of “unstable determination due to noisy acceleration.” Reintroduce or adjust thresholds as necessary.

#### 6.7.3 Retry Conditions

- Falling below the minimum altitude `phase5_height_limit`
- Or reaching a designated WP (e.g., `tkoff_before_cmd`) allows for a “safe retry”

---

### 6.8 Phase 6: Determining TRIM_THROTTLE (Horizontal Steady State)

- Goal: Determine `TRIM_THROTTLE` during the level flight segment where altitude and speed stabilize
- Decision: Adopt when speed and rate of climb fluctuations become sufficiently small within the observation window

---

## **7. Appendix: Minor C++ Modifications for ArduPilot (takeoff.cpp / mode.cpp)**

In this study, in addition to the Lua script-based implementation, **minor C++ modifications for interface compatibility** were made in two locations (without altering TECS's core control laws or PID structures).

### 7.A takeoff.cpp (Mainly additions around lines 194–215)

**Purpose**: Mitigate the tendency for steep pitch commands after rotation during Phase 4 climb when reusing the TAKEOFF command, by scheduling pitch commands smoothly according to speed.

**Overview**:
- After rotation completion (climb phase), using the reference speed `v_ref = AIRSPEED_CRUISE`
- Scale `nav_pitch_cd` according to `EAS / v_ref`
- However, the upper limit is the original `nav_pitch_cd`, and the lower limit is restricted to a safety-side minimum pitch (e.g., 5 deg)
- This prevents excessive nose-up at low speeds, stabilizing reuse during the TAKEOFF phase

### 7. B mode.cpp (Mainly additions around lines 290–318)

**Purpose**: Resolve an issue where, during AUTO mode, if a Lua script manipulates the mission number via `mission:set_current_cmd()`, the ArduPilot's **Altitude Demand may not update immediately**.

**Overview**:
- Under the condition `AP_SCRIPTING_ENABLED`,
- **only in AUTO mode**
- and **when SCR_USER6 (TECS Tune execution flag) is ON**,
- the altitude target for the next command is re-read when the mission number changes,
- and the target altitude is updated via `set_target_altitude_location()`
- `reset_offset_altitude()` also synchronizes the offset altitude
- This ensures altitude commands update in sync with the mission even during Lua-driven phase transitions (WP jumps)

---