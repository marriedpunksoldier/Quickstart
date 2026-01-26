# Flywheel Shooter Testing - Quick Start Guide

## 5-Minute Setup

### 1. Hardware Setup (2 minutes)
- Connect goBILDA 5203 motor to motor port
- Connect encoder cable
- Name motor `"flywheel_motor"` in robot configuration

### 2. Code Installation (2 minutes)
- Copy `FlywheelShooterTest.java` to `teamcode` folder
- Build and deploy to robot
- Find "Flywheel Shooter Test" in TeleOp menu

### 3. First Test (1 minute)
- Start OpMode
- Use **Right Bumper** to increase power to 50%
- Use **D-pad Up** to change distance
- Watch velocity stabilize in telemetry

**You're testing!** ✅

---

## Essential Controls

### Manual Mode (Default)
```
D-pad Up/Down     → Change distance (24" to 72")
Right Bumper      → Increase power by 10%
Left Bumper       → Decrease power by 10%
X Button          → Switch to Menu Mode
```

### PIDF Tuning (Critical for Accuracy)
```
Y + D-pad Up/Down        → Adjust kP
B + Right/Left Bumper    → Adjust kF
```

---

## First-Time PIDF Tuning (10 minutes)

### Starting Point
The code starts with these values:
- kF = 12.0
- kP = 10.0
- kI = 0.1
- kD = 0.0

### Quick Tune Process

**Step 1: Set to 50% power**
- Press Right Bumper 5 times

**Step 2: Tune kF (Feedforward)**
- Look at "Velocity Error" in telemetry
- If error is positive (motor too slow): Press **B + Right Bumper**
- If error is negative (motor too fast): Press **B + Left Bumper**
- Goal: Error within ±100 ticks/sec

**Step 3: Tune kP (Proportional)**
- If error still >50 ticks/sec: Press **Y + D-pad Up**
- If motor oscillates: Press **Y + D-pad Down**
- Goal: Error within ±50 ticks/sec, no bouncing

**Done!** Your shooter should now hold steady velocity.

---

## Testing Workflow

### Quick Distance Test
1. Start in Manual Mode
2. Set power to 70% (Right Bumper × 7)
3. Press D-pad Up to cycle through distances:
   - 24" (2 feet)
   - 36" (3 feet)
   - 48" (4 feet)
   - 60" (5 feet)
   - 72" (6 feet)
4. Record which distances work best

### Systematic Testing (Automatic Mode)
1. Press **X** twice to enter Automatic Mode
2. Press **A** to advance through all combinations
3. System will test all 55 combinations (5 distances × 11 powers)
4. Press **B** to pause when needed

---

## Understanding Telemetry

### What to Watch
```
Distance Preset: 48 inches        ← Current test distance
Power Level: 70%                  ← Current power setting

Current Velocity: 3487 ticks/sec  ← What motor is doing
Target Velocity: 3500 ticks/sec   ← What motor should do
Velocity Error: 13 ticks/sec (0.4%)  ← How far off (lower is better)

kP: 10.50   ← Proportional gain
kF: 12.50   ← Feedforward gain
```

### Good vs. Bad Performance
✅ **Good**: Velocity Error < 50 ticks/sec (< 1%)
❌ **Bad**: Velocity Error > 200 ticks/sec (> 4%)
❌ **Bad**: Velocity bouncing up and down

---

## Common Issues & Fast Fixes

### Motor Not Spinning
- Check power level > 0% (use Right Bumper)
- Verify motor named `"flywheel_motor"` exactly
- Check encoder cable connection

### Velocity Won't Stabilize
- Decrease kP: Press **Y + D-pad Down**
- Increase kF: Press **B + Right Bumper**

### Motor Oscillating (Bouncing)
- Decrease kP: Press **Y + D-pad Down**
- Lower kI to 0: Press **Y + Left Bumper** repeatedly

---

## Data Recording Template

**Quick Test Sheet:**
```
Battery: _____V    Date: _________    PIDF: kP=___ kF=___

Distance | Power | Shots Made | Notes
---------|-------|------------|-------
24"      | 60%   | ___/10     |
36"      | 70%   | ___/10     |
48"      | 80%   | ___/10     |
60"      | 90%   | ___/10     |
72"      | 90%   | ___/10     |
```

---

## 3 Control Modes Explained

### 1. Manual Mode (For Quick Tests)
- Direct control with gamepad
- Fastest for trying specific combinations
- **Best for:** Practice, quick adjustments

### 2. Menu Mode (For Precision)
- Select exact distance and power from list
- Visual confirmation of settings
- **Best for:** Methodical testing

### 3. Automatic Mode (For Complete Testing)
- Tests all 55 combinations automatically
- 3 second delay between tests (adjustable)
- **Best for:** Data collection, mapping performance

Switch modes: Press **X Button**

---

## Safety Reminders

⚠️ **Important:**
1. Keep hands away from spinning flywheel
2. Ensure flywheel is properly mounted
3. Don't leave robot unattended in Automatic Mode
4. Start at low power (30%) and increase gradually
5. Monitor motor temperature during extended testing

---

## Technical Specs (For Reference)

**Motor:** goBILDA 5203 Series
- Max Speed: 6000 RPM
- Encoder: 537.7 ticks/revolution
- At 100% Power: 53,770 ticks/second
- At 50% Power: 26,885 ticks/second

**Test Parameters:**
- Distances: 24", 36", 48", 60", 72"
- Power Levels: 0%, 10%, 20%, ..., 100%
- Total Combinations: 55

---

## Next Steps

### After Initial Testing
1. ✅ Record optimal power for each distance
2. ✅ Save your PIDF values in notebook
3. ✅ Test at different battery voltages
4. ✅ Practice using your optimal settings

### For Complete Information
- Read `SHOOTER_TESTING_DOCUMENTATION.md` for:
  - Detailed PIDF tuning guide
  - Advanced features
  - Troubleshooting
  - Technical calculations

---

## Keyboard Shortcut Reference Card

**Print and tape to driver station!**

```
┌─────────────────────────────────────────┐
│  FLYWHEEL SHOOTER TESTING - CONTROLS    │
├─────────────────────────────────────────┤
│  MODE SWITCHING                         │
│  X ───────────── Switch Mode            │
├─────────────────────────────────────────┤
│  MANUAL MODE                            │
│  D-pad Up/Down ── Change Distance       │
│  Right Bumper ─── +10% Power            │
│  Left Bumper ──── -10% Power            │
├─────────────────────────────────────────┤
│  AUTOMATIC MODE                         │
│  A ─────────────── Advance Now          │
│  B ─────────────── Pause/Resume         │
├─────────────────────────────────────────┤
│  PIDF TUNING (ALL MODES)                │
│  Y + D-pad ────── Adjust kP             │
│  Y + Bumpers ──── Adjust kI             │
│  B + D-pad ────── Adjust kD             │
│  B + Bumpers ──── Adjust kF             │
└─────────────────────────────────────────┘
```

---

**Ready to test? Start the OpMode and press Right Bumper to begin!**

For questions or issues, see the full documentation or consult your team mentors.
