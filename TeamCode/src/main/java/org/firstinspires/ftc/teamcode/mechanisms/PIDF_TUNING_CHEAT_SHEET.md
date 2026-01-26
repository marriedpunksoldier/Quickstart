# PIDF Tuning Cheat Sheet
## Quick Reference for Flywheel Shooter Testing

---

## PIDF Coefficients Overview

| Coefficient | Purpose | Starting Value | Typical Range |
|-------------|---------|----------------|---------------|
| **kF** | Base power for target speed | 12.0 | 10.0 - 15.0 |
| **kP** | Error correction strength | 10.0 | 5.0 - 20.0 |
| **kI** | Eliminate steady error | 0.1 | 0.0 - 0.5 |
| **kD** | Dampen oscillations | 0.0 | 0.0 - 5.0 |

---

## Tuning Order (CRITICAL!)

```
1. kF (Feedforward)    ← Start here
2. kP (Proportional)   ← Then this
3. kD (Derivative)     ← If needed
4. kI (Integral)       ← Last resort only
```

**Never start with kP!** Always tune kF first.

---

## Step-by-Step Tuning Process

### STEP 1: Tune kF (Feedforward)
**Set motor to 50% power**

| Observation | Action | Button Press |
|-------------|--------|--------------|
| Motor too slow (positive error) | Increase kF | B + Right Bumper |
| Motor too fast (negative error) | Decrease kF | B + Left Bumper |

**Goal:** Velocity Error within ±100 ticks/sec

**How long?** 2-5 minutes of adjustment

---

### STEP 2: Tune kP (Proportional)
**Keep kF from Step 1**

| Observation | Action | Button Press |
|-------------|--------|--------------|
| Still too much error (>50 ticks/sec) | Increase kP | Y + D-pad Up |
| Velocity oscillating/bouncing | Decrease kP | Y + D-pad Down |

**Goal:** Velocity Error within ±50 ticks/sec, no oscillation

**How long?** 1-3 minutes of adjustment

---

### STEP 3: Tune kD (Derivative) - IF NEEDED
**Only if velocity is oscillating**

| Observation | Action | Button Press |
|-------------|--------|--------------|
| Velocity ringing/bouncing | Increase kD | B + D-pad Up |
| Response too sluggish | Decrease kD | B + D-pad Down |

**Goal:** Smooth response, no overshoot

**How long?** 1-2 minutes or skip if no oscillation

---

### STEP 4: Tune kI (Integral) - RARELY NEEDED
**⚠️ Use sparingly! Can cause instability**

| Observation | Action | Button Press |
|-------------|--------|--------------|
| Small persistent error won't go away | Increase kI slightly | Y + Right Bumper |
| Velocity unstable, wind-up | Decrease kI or set to 0 | Y + Left Bumper |

**Goal:** Zero steady-state error without instability

**How long?** Skip entirely if error < 50 ticks/sec

---

## Visual Diagnosis Guide

### Velocity Error Patterns

```
GOOD - Well Tuned:
Target: ═══════════════════════════════
Actual: ═══════════════════════════════
Error:  Small, stable (< 50 ticks/sec)

BAD - kF too low:
Target: ═══════════════════════════════
Actual: ──────────────────────────────
Error:  Consistently positive (motor slow)
FIX: B + Right Bumper (increase kF)

BAD - kF too high:
Target: ═══════════════════════════════
Actual: ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
Error:  Consistently negative (motor fast)
FIX: B + Left Bumper (decrease kF)

BAD - kP too high:
Target: ═══════════════════════════════
Actual: ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
Error:  Oscillating (bouncing)
FIX: Y + D-pad Down (decrease kP)

BAD - kI too high:
Target: ═══════════════════════════════
Actual: ╱▔╲_╱▔╲_╱▔╲_╱▔╲_╱▔╲_╱▔╲_
Error:  Unstable, slow oscillations
FIX: Y + Left Bumper (decrease kI to 0)
```

---

## Button Reference

### PIDF Tuning Controls

```
┌────────────────────────────────────┐
│  Y + D-pad Up    │ kP ↑ (+0.5)     │
│  Y + D-pad Down  │ kP ↓ (-0.5)     │
├────────────────────────────────────┤
│  Y + Right Bump  │ kI ↑ (+0.05)    │
│  Y + Left Bump   │ kI ↓ (-0.05)    │
├────────────────────────────────────┤
│  B + D-pad Up    │ kD ↑ (+0.1)     │
│  B + D-pad Down  │ kD ↓ (-0.1)     │
├────────────────────────────────────┤
│  B + Right Bump  │ kF ↑ (+0.5)     │
│  B + Left Bump   │ kF ↓ (-0.5)     │
└────────────────────────────────────┘
```

---

## Telemetry Quick Read

### What to Watch During Tuning

```
Motor Performance Section:
┌─────────────────────────────────────────┐
│ Current Velocity: 3487 ticks/sec        │ ← Actual
│ Target Velocity:  3500 ticks/sec        │ ← Goal
│ Velocity Error:   13 ticks/sec (0.4%)   │ ← How far off
└─────────────────────────────────────────┘
                     ↑
              Watch this number!
              Goal: < 50 ticks/sec (< 1%)
```

---

## Adjustment Increments

| Coefficient | Per Button Press | When to Adjust |
|-------------|------------------|----------------|
| kF | ±0.5 | Error > 100 ticks/sec |
| kP | ±0.5 | Error > 50 ticks/sec |
| kD | ±0.1 | Oscillations present |
| kI | ±0.05 | Persistent small error |

---

## Troubleshooting Decision Tree

```
START
  │
  ├─ Error > 100 ticks/sec?
  │   YES → Adjust kF
  │   NO  → Go to next check
  │
  ├─ Error > 50 ticks/sec?
  │   YES → Increase kP
  │   NO  → Go to next check
  │
  ├─ Velocity oscillating?
  │   YES → Decrease kP or increase kD
  │   NO  → Go to next check
  │
  └─ Small persistent error?
      YES → Slightly increase kI (careful!)
      NO  → ✓ TUNING COMPLETE
```

---

## Common Mistakes to Avoid

❌ **DON'T:**
- Start by tuning kP (always start with kF)
- Increase kI before kF and kP are tuned
- Make large adjustments (use small increments)
- Tune at 100% power initially (use 50%)
- Ignore oscillations (they damage motors)

✅ **DO:**
- Tune kF first, always
- Test across multiple power levels after initial tuning
- Record successful values before changes
- Allow motor to stabilize (2-3 seconds) before judging
- Retest after battery voltage changes

---

## Example Tuning Session

### Starting Values
```
kF = 12.0
kP = 10.0
kI = 0.1
kD = 0.0
```

### Tuning Log Example
```
Power: 50% (Target: 26,885 ticks/sec)

Time | Action         | kF   | kP   | Error    | Notes
-----|----------------|------|------|----------|------------------
0:00 | Initial        | 12.0 | 10.0 | +200     | Too slow
0:30 | B+RightBump ×2 | 13.0 | 10.0 | +50      | Better
1:00 | B+RightBump    | 13.5 | 10.0 | +20      | Close
1:30 | Y+D-pad Up     | 13.5 | 10.5 | +5       | Good!
2:00 | Test @ 70%     | 13.5 | 10.5 | +8       | Stable
2:30 | Test @ 90%     | 13.5 | 10.5 | +12      | ✓ COMPLETE
```

**Final Values:** kF=13.5, kP=10.5, kI=0.1, kD=0.0

---

## Battery Voltage Considerations

### Effect on Performance
| Battery Level | Voltage | Effect on Motor |
|---------------|---------|-----------------|
| Fresh | 13.5V | Fastest, may need lower kF |
| Half | 12.8V | Nominal performance |
| Low | 12.2V | Slower, may need higher kF |

**Recommendation:** Tune at ~13.0V (mid-range battery)

**Competition Strategy:** 
- Note kF at different voltages
- Adjust kF during match if needed
- Test range: 12.5V to 13.5V

---

## Success Criteria

### Well-Tuned System Shows:
- ✅ Velocity error < 1% of target
- ✅ No visible oscillations
- ✅ Quick recovery from load changes
- ✅ Consistent across 30%-90% power range
- ✅ Stable across battery voltage range
- ✅ Repeatable shot accuracy

### Time Investment:
- **Initial Tune:** 5-10 minutes
- **Fine Tune:** 5-15 minutes
- **Validation:** 10-20 minutes
- **Total:** ~30-45 minutes for complete tuning

---

## Recording Template

**Print this section for testing:**

```
═══════════════════════════════════════════
FLYWHEEL PIDF TUNING LOG
═══════════════════════════════════════════

Date: ___________  Battery: _____V

Initial Values:
  kF = _____   kP = _____   kI = _____   kD = _____

Power Level for Tuning: _____%
Target Velocity: __________ ticks/sec

Tuning Changes:
Time | Action | kF  | kP  | kI  | kD  | Error | Notes
_____|________|_____|_____|_____|_____|_______|_______
     |        |     |     |     |     |       |
     |        |     |     |     |     |       |
     |        |     |     |     |     |       |
     |        |     |     |     |     |       |

Final Tuned Values:
  kF = _____   kP = _____   kI = _____   kD = _____

Validation Tests (Final Values):
Power | Target  | Actual  | Error | Pass?
______|_________|_________|_______|_______
30%   |         |         |       | Y / N
50%   |         |         |       | Y / N
70%   |         |         |       | Y / N
90%   |         |         |       | Y / N

Battery Voltage Tests:
Voltage | kF needed | Notes
________|___________|_______
13.5V   |           |
13.0V   |           |
12.5V   |           |

Competition Values (COPY TO ROBOT):
═══════════════════════════════════════════
  kF = _____   kP = _____   kI = _____   kD = _____
═══════════════════════════════════════════

Tuned by: _______________  Verified by: _______________
```

---

## Emergency Reset

**If system becomes unstable:**

1. Set all to safe defaults:
   - kF = 12.0
   - kP = 5.0
   - kI = 0.0
   - kD = 0.0

2. Reduce power to 30%

3. Start tuning process from beginning

4. Document what went wrong in notebook

---

## Quick Reference - At-A-Glance

```
┌───────────────────────────────────────────────┐
│ TUNING SEQUENCE                               │
├───────────────────────────────────────────────┤
│ 1. Set to 50% power                           │
│ 2. Adjust kF until error ≈ ±100 ticks/sec     │
│ 3. Adjust kP until error < 50 ticks/sec       │
│ 4. If oscillating, add kD                     │
│ 5. If persistent error, add tiny kI           │
│ 6. Test at 30%, 50%, 70%, 90%                 │
│ 7. Record final values                        │
└───────────────────────────────────────────────┘

ERROR DIAGNOSIS:
  +200 ticks/sec → Increase kF (B + Right Bump)
  -200 ticks/sec → Decrease kF (B + Left Bump)
   +50 ticks/sec → Increase kP (Y + D-pad Up)
   Oscillating   → Decrease kP (Y + D-pad Down)
   Still bouncing → Increase kD (B + D-pad Up)

TARGET: < 50 ticks/sec error (< 1%)
```

---

**Print this sheet and keep at driver station during tuning sessions!**

For complete information, see SHOOTER_TESTING_DOCUMENTATION.md
