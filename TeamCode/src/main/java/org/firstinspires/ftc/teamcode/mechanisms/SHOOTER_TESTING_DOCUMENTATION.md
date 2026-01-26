# Flywheel Shooter Testing System - Documentation

## Overview
This comprehensive testing system allows you to systematically test, calibrate, and optimize your FTC flywheel shooter mechanism across 5 distance presets and 11 power levels with real-time PIDF tuning capabilities.

---

## Hardware Requirements

### Motor Specifications
- **Motor Model**: goBILDA 5203 Series Yellow Jacket Planetary Gear Motor
- **Maximum RPM**: 6000 RPM
- **Encoder Resolution**: 537.7 ticks per revolution
  - Calculation: 28 counts per motor shaft revolution × 19.2:1 gear ratio
- **Configuration Name in Robot**: `"flywheel_motor"`

### Hardware Setup
1. Mount the goBILDA 5203 motor to your flywheel mechanism
2. Connect the motor to a motor port on your Control Hub or Expansion Hub
3. Ensure the encoder cable is properly connected
4. Configure the motor in your robot configuration file with the name `"shooter"`

---

## Installation & Setup

### Step 1: Add Code to Your Project
1. Copy `FlywheelShooterTest.java` to your FTC project's `teamcode` folder
2. Path should be: `FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FlywheelShooterTest.java`

### Step 2: Configure Robot Hardware
1. On your Driver Station, go to the configuration menu
2. Create or edit your robot configuration
3. Add a DC Motor named exactly `"shooter"` on the appropriate port
4. Save the configuration

### Step 3: Build and Deploy
1. Build your project in Android Studio
2. Deploy to your Control Hub/Robot Controller
3. The OpMode will appear in TeleOp as "Flywheel Shooter Test"

---

## Testing Parameters

### Distance Presets (5 Total)
| Preset # | Distance (inches) | Distance (feet) |
|----------|------------------|-----------------|
| 1        | 24               | 2               |
| 2        | 36               | 3               |
| 3        | 48               | 4               |
| 4        | 60               | 5               |
| 5        | 72               | 6               |

### Power Levels (11 Total)
0%, 10%, 20%, 30%, 40%, 50%, 60%, 70%, 80%, 90%, 100%

### Total Test Combinations
5 distances × 11 power levels = **55 unique combinations**

---

## Control Modes

### 1. Manual Mode (Default)
**Purpose**: Direct control for quick testing and adjustments

**Controls**:
- **D-pad Up/Down**: Cycle through distance presets (24" → 36" → 48" → 60" → 72" → 24")
- **Left Bumper**: Decrease power by 10% (minimum 0%)
- **Right Bumper**: Increase power by 10% (maximum 100%)
- **X Button**: Switch to Menu Mode

**Best For**:
- Quick testing of specific combinations
- Real-time adjustments during practice
- Immediate response to observations

---

### 2. Menu Mode
**Purpose**: Number-based selection for precise parameter choice

**Controls**:
- **D-pad Up/Down**: Navigate menu items
- **A Button**: Select/cycle highlighted item
  - Distance: Cycles through 24" → 36" → 48" → 60" → 72" → 24"
  - Power: Cycles through 0% → 10% → 20% → ... → 100% → 0%
- **X Button**: Switch to Automatic Mode

**Menu Display**:
```
Menu Selection:
  ► Distance: 48 inches     (Selected item marked with ►)
    Power: 70%
```

**Best For**:
- Methodical testing with specific parameter selection
- Returning to previously tested combinations
- Clear visual confirmation of settings

---

### 3. Automatic Cycling Mode
**Purpose**: Systematic testing of all combinations

**Controls**:
- **A Button**: Manually advance to next combination
- **B Button**: Pause/Resume automatic cycling
- **X Button**: Switch to Manual Mode

**Cycling Pattern**:
1. Tests all 11 power levels at 24"
2. Advances to 36" and tests all 11 power levels
3. Continues through 48", 60", and 72"
4. Loops back to start (24", 0%)

**Timing**:
- Automatic advance: Every 3 seconds (when not paused)
- Manual advance: Immediate when A is pressed

**Progress Display**:
```
Auto Status: RUNNING
Progress: 23/55 combinations
Next advance in: 1.7s
```

**Best For**:
- Comprehensive data collection
- Systematic performance mapping
- Unattended testing sessions (with supervision)

---

## PIDF Tuning System

### What is PIDF?
PIDF (Proportional-Integral-Derivative-Feedforward) control ensures the flywheel maintains consistent velocity despite load variations and battery voltage changes.

### PIDF Components
- **kF (Feedforward)**: Base power needed to maintain velocity
- **kP (Proportional)**: Corrects based on current error
- **kD (Derivative)**: Dampens oscillations and overshooting
- **kI (Integral)**: Eliminates steady-state error (use sparingly)

### Tuning Controls (Available in All Modes)
| Coefficient | Increase | Decrease |
|-------------|----------|----------|
| **kP**      | Y + D-pad Up | Y + D-pad Down |
| **kI**      | Y + Right Bumper | Y + Left Bumper |
| **kD**      | B + D-pad Up* | B + D-pad Down* |
| **kF**      | B + Right Bumper* | B + Left Bumper* |

*In Automatic Mode: Only works when paused

### Adjustment Increments
- kP: ±0.5 per button press
- kI: ±0.05 per button press
- kD: ±0.1 per button press
- kF: ±0.5 per button press

---

## Step-by-Step Tuning Guide

### Starting Values (Pre-configured)
```
kF = 12.0   (Feedforward - calculated from motor specs)
kP = 10.0   (Proportional)
kI = 0.1    (Integral)
kD = 0.0    (Derivative)
```

### Tuning Process

#### Step 1: Tune kF (Feedforward)
**Goal**: Get close to target velocity with minimal oscillation

1. Set power to 50% in Manual Mode
2. Observe "Velocity Error" in telemetry
3. If error is consistently positive (motor too slow):
   - Press B + Right Bumper to increase kF
4. If error is consistently negative (motor too fast):
   - Press B + Left Bumper to decrease kF
5. Repeat until error is within ±100 ticks/sec most of the time

**Target**: Small, stable error without large swings

---

#### Step 2: Tune kP (Proportional)
**Goal**: Reduce remaining error without causing oscillation

1. Keep current kF value
2. If velocity error is still too large:
   - Press Y + D-pad Up to increase kP
3. Watch for oscillation (velocity bouncing up and down)
4. If oscillation occurs:
   - Press Y + D-pad Down to decrease kP slightly
5. Find the highest kP that doesn't cause oscillation

**Target**: Error < ±50 ticks/sec without bouncing

---

#### Step 3: Tune kD (Derivative)
**Goal**: Dampen any remaining oscillations

1. Only needed if you see velocity "ringing" or bouncing
2. Press B + D-pad Up to increase kD gradually
3. Stop when oscillations are eliminated
4. Too much kD will make the system sluggish

**Target**: Smooth velocity response without overshoot

---

#### Step 4: Tune kI (Integral) - Optional
**Goal**: Eliminate persistent steady-state error

⚠️ **Warning**: Start with kI = 0 and only increase if absolutely needed

1. Only tune kI if there's a consistent small error that won't go away
2. Press Y + Right Bumper to increase kI very slightly (0.05 at a time)
3. Too much kI will cause instability and wind-up
4. Typical final values: 0.0 to 0.3

**Target**: Zero steady-state error without instability

---

### Tuning Tips

#### Battery Voltage Matters
- Higher battery voltage = motor spins faster
- Test PIDF at various battery levels (12.5V, 13.0V, 13.5V)
- kF may need adjustment as battery drains

#### Signs of Good Tuning
- ✅ Velocity error < ±50 ticks/sec (±1% of target)
- ✅ No oscillations or bouncing
- ✅ Quick recovery from disturbances
- ✅ Consistent performance across power levels

#### Signs of Poor Tuning
- ❌ Large persistent error (>200 ticks/sec)
- ❌ Velocity bouncing up and down
- ❌ Slow response to power changes
- ❌ Unstable at high speeds

---

## Telemetry Reference

### Telemetry Display Structure
```
═══════════════════════════════════════
║ CONTROL MODE: MANUAL
═══════════════════════════════════════

[Mode-specific controls and information]

───────────────────────────────────────
CURRENT TEST PARAMETERS
───────────────────────────────────────
Distance Preset: 48 inches
Power Level: 70%

───────────────────────────────────────
MOTOR PERFORMANCE
───────────────────────────────────────
Current Velocity: 3487 ticks/sec
Target Velocity: 3500 ticks/sec
Current RPM: 3890 RPM
Target RPM: 3905 RPM
Velocity Error: 13 ticks/sec (0.4%)
Motor Power: 0.72

───────────────────────────────────────
PIDF COEFFICIENTS
───────────────────────────────────────
kP: 10.50  (Y + D-pad)
kI: 0.100  (Y + Bumpers)
kD: 0.00  (B + D-pad)
kF: 12.50  (B + Bumpers)

───────────────────────────────────────
Runtime: 45.3 seconds
Motor: goBILDA 5203 (6000 RPM)
```

### Key Metrics Explained

#### Velocity Metrics
- **Current Velocity**: Actual motor speed (encoder ticks per second)
- **Target Velocity**: Desired motor speed based on power setting
- **Current RPM**: Actual motor speed (revolutions per minute)
- **Target RPM**: Desired motor speed in RPM

#### Error Analysis
- **Velocity Error (ticks/sec)**: Difference between target and actual
  - Positive = Motor too slow
  - Negative = Motor too fast
- **Error Percentage**: Relative error as percentage of target
  - Goal: Keep below 1-2% for consistent shooting

#### Motor Power
- Actual power output from PIDF controller (0.0 to 1.0)
- Will vary as PIDF compensates for error
- Should stabilize near the power level setting when tuned correctly

---

## Testing Methodology

### Recommended Testing Sequence

#### Phase 1: Initial Calibration (Manual Mode)
1. Start with 50% power at 48" distance
2. Tune PIDF coefficients following the step-by-step guide
3. Test at 30%, 50%, 70%, 90% power to verify tuning across range
4. Record any power levels where performance degrades

#### Phase 2: Distance Mapping (Menu Mode)
1. Set power to optimal level (likely 70-90%)
2. Test each distance preset: 24", 36", 48", 60", 72"
3. Record shooting accuracy at each distance
4. Note any distances requiring power adjustment

#### Phase 3: Comprehensive Testing (Automatic Mode)
1. Pause automatic cycling initially (B button)
2. Position targets at each distance
3. Resume automatic cycling
4. Record success rate for each distance-power combination
5. Use A button to repeat specific combinations as needed

#### Phase 4: Competition Optimization
1. Based on Phase 3 data, identify optimal power levels for each distance
2. Test battery voltage effects (fresh vs. partially drained battery)
3. Verify PIDF stability across entire competition battery range
4. Document final settings in engineering notebook

---

## Data Collection Template

### Recording Sheet
```
Date: ___________  Battery Voltage: _____V  PIDF: kP=___ kI=___ kD=___ kF=___

Distance | Power | Target RPM | Actual RPM | Error % | Shots Made | Notes
---------|-------|------------|------------|---------|------------|-------
24"      | 50%   |            |            |         | ___/10     |
24"      | 60%   |            |            |         | ___/10     |
...
72"      | 90%   |            |            |         | ___/10     |
```

### Key Data Points to Record
- Battery voltage at start and end of testing
- PIDF coefficients used
- Target vs. actual RPM for each combination
- Shooting accuracy (successful shots out of 10 attempts)
- Environmental conditions (temperature, humidity if relevant)
- Any unusual behaviors or observations

---

## Troubleshooting

### Motor Won't Spin
**Symptoms**: Zero velocity, no movement
**Checks**:
1. Verify motor is plugged into correct port
2. Check that motor name in code matches configuration exactly: `"flywheel_motor"`
3. Ensure encoder cable is connected
4. Confirm motor is set to RUN_USING_ENCODER mode
5. Try increasing power level above 0%

---

### Velocity Unstable / Oscillating
**Symptoms**: RPM bouncing up and down rapidly
**Solutions**:
1. Decrease kP: Press Y + D-pad Down
2. Increase kD: Press B + D-pad Up (if not already tuned)
3. Reduce kI to 0: Press Y + Left Bumper repeatedly
4. Check for mechanical binding or friction in flywheel

---

### Persistent Velocity Error
**Symptoms**: Consistently off target by >100 ticks/sec
**Solutions**:
1. Adjust kF: 
   - Too slow → B + Right Bumper
   - Too fast → B + Left Bumper
2. Check battery voltage (low battery = slower speeds)
3. Verify motor specifications match code (6000 RPM, 537.7 ticks/rev)

---

### Mode Not Switching
**Symptoms**: X button doesn't change modes
**Solutions**:
1. Ensure you're pressing and releasing X (not holding)
2. Check for gamepad connectivity issues
3. Verify only one button is being pressed at a time
4. Restart OpMode if issue persists

---

### Automatic Mode Not Advancing
**Symptoms**: Stuck on one combination
**Checks**:
1. Verify mode shows "RUNNING" not "PAUSED"
2. If paused, press B to resume
3. Check "Next advance in" timer is counting down
4. Manually advance with A button to test responsiveness

---

## Technical Specifications

### Motor Calculations
```
Motor: goBILDA 5203 Series (6000 RPM max)
Encoder: 537.7 ticks per revolution
  = 28 CPR (motor shaft) × 19.2:1 (gear ratio)

At 100% Power (6000 RPM):
  Ticks per second = (6000 RPM / 60 sec) × 537.7 ticks/rev
                    = 100 rev/sec × 537.7 ticks/rev
                    = 53,770 ticks/sec

At 50% Power (3000 RPM):
  Ticks per second = (3000 RPM / 60 sec) × 537.7 ticks/rev
                    = 50 rev/sec × 537.7 ticks/rev
                    = 26,885 ticks/sec
```

### Velocity Control
- **Control Mode**: RUN_USING_ENCODER with velocity PID
- **Update Rate**: ~20ms per cycle (50 Hz)
- **Measurement**: Direct encoder feedback
- **Output**: Power compensation based on PIDF algorithm

### Power Level Mapping
| Power % | RPM Target | Ticks/Second |
|---------|-----------|--------------|
| 0%      | 0         | 0            |
| 10%     | 600       | 5,377        |
| 20%     | 1,200     | 10,754       |
| 30%     | 1,800     | 16,131       |
| 40%     | 2,400     | 21,508       |
| 50%     | 3,000     | 26,885       |
| 60%     | 3,600     | 32,262       |
| 70%     | 4,200     | 37,639       |
| 80%     | 4,800     | 43,016       |
| 90%     | 5,400     | 48,393       |
| 100%    | 6,000     | 53,770       |

---

## Safety & Best Practices

### Safety Guidelines
1. ⚠️ **Never run motor at full power without proper mounting** - flywheel can cause injury
2. ⚠️ **Ensure flywheel is properly enclosed** - prevent contact during operation
3. ⚠️ **Monitor motor temperature** - extended high-speed operation can overheat motor
4. ⚠️ **Supervise automatic mode** - don't leave robot unattended during cycling
5. ⚠️ **Clear shooting area** - ensure no people or obstacles in trajectory path

### Best Practices
1. ✅ Start testing at low power levels (30-40%) and increase gradually
2. ✅ Allow motor to stabilize for 2-3 seconds before recording data
3. ✅ Test with fully charged battery for baseline, then retest at lower voltages
4. ✅ Record all PIDF changes in engineering notebook
5. ✅ Save successful PIDF coefficients before making major changes
6. ✅ Clean flywheel wheels regularly - debris affects consistency
7. ✅ Check motor mounting screws periodically - vibration can loosen them

---

## Advanced Features

### Custom Distance Presets
To modify distance presets, edit this line in code:
```java
private final int[] DISTANCE_PRESETS = {24, 36, 48, 60, 72};
```
Example for different distances:
```java
private final int[] DISTANCE_PRESETS = {18, 30, 42, 54, 66}; // Custom values
```

### Custom Power Increments
To use 5% increments instead of 10%, edit:
```java
private final double[] POWER_LEVELS = {
    0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0
};
```

### Automatic Cycling Speed
To change delay between automatic advances, edit:
```java
private static final double AUTO_ADVANCE_DELAY = 3.0; // seconds
```
Example for 5-second delays:
```java
private static final double AUTO_ADVANCE_DELAY = 5.0; // seconds
```

---

## Quick Reference Card

### Mode Switching
- **X**: Switch modes (Manual → Menu → Automatic → Manual)

### Manual Mode Controls
- **D-pad Up/Down**: Change distance
- **Left/Right Bumper**: Adjust power

### Menu Mode Controls
- **D-pad Up/Down**: Navigate menu
- **A**: Select item

### Automatic Mode Controls
- **A**: Advance now
- **B**: Pause/Resume

### PIDF Tuning (All Modes)
- **Y + D-pad**: Adjust kP
- **Y + Bumpers**: Adjust kI
- **B + D-pad**: Adjust kD (not in auto unless paused)
- **B + Bumpers**: Adjust kF (not in auto unless paused)

---

## Support & Resources

### FTC Documentation
- FTC SDK Documentation: https://ftc-docs.firstinspires.org/
- goBILDA Motor Specs: https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor/

### Additional Help
1. Check FTC Discord: https://discord.gg/first-tech-challenge
2. Review Game Manual Part 1 for rules on motor usage
3. Consult with mentors or experienced teams
4. Document all issues in engineering notebook

---

## Version History

**Version 1.0** - Initial Release
- 3 control modes (Manual, Menu, Automatic)
- 5 distance presets × 11 power levels = 55 combinations
- Real-time PIDF tuning
- Comprehensive telemetry
- goBILDA 5203 motor support

---

## Credits

Created for FTC Decode Season
Motor: goBILDA 5203 Series (6000 RPM)
Framework: FTC SDK LinearOpMode

---

**Good luck with your flywheel testing!**

Remember: Systematic testing and careful documentation are the keys to competition success. Take your time with PIDF tuning, record all your findings, and iterate based on real-world results.
