essw# Controller Reference — Teleop

Two gamepads. Gamepad 1 drives, Gamepad 2 operates the shooter system.
Buttons not listed below have no function in teleop.

## GAMEPAD 1 — Driver

| Control | Function |
|---|---|
| **Left stick** | Drive: forward / back / strafe. Robot-centric until localizer is ready, then field-centric. Radial deadband + squared curve for fine control. |
| **Right stick (X)** | Rotate the robot in place. |
| **B button** | Auto-move to scoring distance. Press once; robot drives toward the visible AprilTag and rotates to face it. Requires a tag in view. Cancels if you move the stick more than 15% or after 2.5 s. |

## GAMEPAD 2 — Operator

### Shooter

| Control | Function |
|---|---|
| **Right bumper (RB)** | Spin shooter UP. |
| **X button** | Spin shooter DOWN. |
| **Right trigger (RT)** | Fire one ball. Edge-triggered — pull RT once per shot. Only fires when shooter is at speed AND turret is locked on AND tag is visible. Holding RT does NOT spray balls. |
| **Y button** | Toggle auto-distance mode. ON (default) = Limelight distance auto-selects the shooter RPM. OFF = use manual preset (D-pad). |

### Distance Preset (when auto-distance is OFF)

| Control | Function |
|---|---|
| **D-pad UP** | Next preset (longer shot). 8 presets total: 2ft / 3ft / 4ft / 5ft / 6ft / 7ft / 10ft / 11ft. |
| **D-pad DOWN** | Previous preset (shorter shot). |

### Intake

| Control | Function |
|---|---|
| **Left bumper (LB)** | Intake both motors at full power (collect balls from field). |
| **Left trigger (LT)** | Proportional outtake — both motors reversed by trigger amount. Use to clear a jam. |
| *(automatic)* | During firing (stopper open), rear intake automatically pulses to push the ball into the shooter. Driver intake input is ignored during the ~250 ms firing window. |

## Driver Feedback

| Signal | Meaning |
|---|---|
| **Indicator LED OFF** | No AprilTag in view. |
| **Indicator LED GREEN** | AprilTag visible. Driver knows the aim is good. |
| **Gamepad 2 rumble** | Full ready-to-fire: tag visible + shooter at speed + turret locked. Pull RT now. One-shot pulse on transition. |

## Telemetry Diagnostics

If something isn't working, the Driver Station telemetry will show:

| Section | Tells you |
|---|---|
| **Loop ms** | How fast each loop is running. Target: <30 ms. >50 ms means something is bottlenecked. |
| **Shooter** | Target RPM, actual RPM, motor output, ready flag. Motor output should sit around 0.55-0.70 when settled at target. |
| **Limelight: Pipeline / Target ID** | What we're configured to look for. Verify it matches your robot config and field. |
| **Limelight: Staleness** | How old the latest result is in ms. Should be <100 ms. |
| **Limelight: Visible tags** | What the camera actually sees right now. If empty when you expect a tag, the camera isn't seeing it — check pipeline, exposure, distance. |
| **Limelight: State** | If "no target", says why. Common reasons: "result == null" (camera not streaming), "no fiducials in result" (camera sees no tags), "target tag X not in visible set" (camera sees other tags but not yours). |
| **Turret: Locked on** | YES when within 2 degrees of perfect alignment. Required for firing. |
| **Pose** | Current robot field position from the localizer. Says "localizer not ready" until first odometry update. |
| **Auto-move state** | IDLE or MOVING. Errors during auto-move appear here. |
| **READY TO FIRE** | The combined gate. *** YES *** means RT will fire a ball. |
