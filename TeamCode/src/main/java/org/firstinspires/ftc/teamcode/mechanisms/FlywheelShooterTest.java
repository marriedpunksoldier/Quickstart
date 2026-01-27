package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FTC Flywheel Shooter Testing System
 * 
 * This OpMode provides comprehensive testing and tuning capabilities for a flywheel shooter
 * mechanism using a goBILDA 5203 series 6000 RPM motor.
 *
 * Features:
 * - 5 distance presets (24, 36, 48, 60, 72, 84, 96, 108, 120 inches)
 * - 11 power levels (0% to 100% in 10% increments)
 * - 3 control modes: Manual, Menu, and Automatic Cycling
 * - Real-time PIDF coefficient tuning
 * - Velocity-based motor control
 * - Comprehensive telemetry display
 * 
 * Controls:
 * - X button: Switch between Manual, Menu, and Automatic modes
 * - Manual Mode:
 *   - D-pad Up/Down: Cycle through distance presets
 *   - Left/Right Bumper: Decrease/Increase power level
 * - Menu Mode:
 *   - D-pad Up/Down: Navigate menu options
 *   - A button: Select highlighted option
 * - Automatic Mode:
 *   - A button: Advance to next distance-power combination
 *   - B button: Pause/Resume automatic cycling
 * - PIDF Tuning (all modes):
 *   - Y + D-pad Up/Down: Adjust kP
 *   - Y + Left/Right Bumper: Adjust kI
 *   - B + D-pad Up/Down: Adjust kD
 *   - B + Left/Right Bumper: Adjust kF
 */
@TeleOp(name = "Flywheel Shooter Test", group = "Testing")
public class FlywheelShooterTest extends LinearOpMode {
    
    // Hardware
    private DcMotorEx shooter;
    private Servo pusherServo;

    // Pusher servo positions
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;
    private boolean pusherExtended = false;

    // Motor specifications for goBILDA 5203 series
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_TICKS_PER_REV = 28.0; // 28 CPR × 1:1 ratio
    private static final double TICKS_PER_SECOND_AT_MAX_RPM = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;
    
    // Distance presets in inches
    private final int[] DISTANCE_PRESETS = {24, 36, 48, 60, 72, 84, 96, 108, 120}; // 2ft, 3ft, 4ft, 5ft, 6ft, 7ft, 8ft, 9ft, 10ft
    
    // Power levels (0% to 100% in 10% increments)
    private final double[] POWER_LEVELS = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0};
    
    // Current test settings
    private int currentDistanceIndex = 0;
    private int currentPowerIndex = 0;
    
    // Control mode enum
    private enum ControlMode {
        MANUAL,
        MENU,
        AUTOMATIC
    }
    
    private ControlMode currentMode = ControlMode.MANUAL;
    
    // Automatic cycling variables
    private int autoDistanceIndex = 0;
    private int autoPowerIndex = 0;
    private boolean autoPaused = false;
    private boolean autoAdvancePressed = false;
    
    // Menu mode variables
    private int menuSelection = 0;
    private final int MENU_ITEMS = 2; // Distance and Power selection
    
    // PIDF coefficients - starting values (will need tuning)
    private double kP = 9.0;
    private double kI = 0.100;
    private double kD = 0.1;
    private double kF = 12.5; // Start with feedforward based on motor characteristics
    
    // PIDF adjustment increments
    private static final double KP_INCREMENT = 0.5;
    private static final double KI_INCREMENT = 0.05;
    private static final double KD_INCREMENT = 0.1;
    private static final double KF_INCREMENT = 0.5;
    
    // Button state tracking for debouncing
    private boolean lastXButton = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastRightTrigger = false;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime autoModeTimer = new ElapsedTime();
    private static final double AUTO_ADVANCE_DELAY = 3.0; // seconds between automatic advances
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Display initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor", "goBILDA 5203 (6000 RPM)");
        telemetry.addData("Mode", "Manual");
        telemetry.addLine("\nPress Start to begin testing");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Main control loop
        while (opModeIsActive()) {
            // Handle mode switching
            handleModeSwitch();
            
            // Handle PIDF tuning (available in all modes)
            handlePIDFTuning();
            
            // Process controls based on current mode
            switch (currentMode) {
                case MANUAL:
                    handleManualMode();
                    break;
                case MENU:
                    handleMenuMode();
                    break;
                case AUTOMATIC:
                    handleAutomaticMode();
                    break;
            }
            
            // Update motor based on current settings
            updateMotor();

            // Handle pusher servo
            handlePusher();

            // Display telemetry
            displayTelemetry();
            
            telemetry.update();
        }
        
        // Stop motor when OpMode ends
        shooter.setPower(0);
    }
    
    /**
     * Initialize hardware components
     */
    private void initializeHardware() {
        // Configure flywheel motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure pusher servo
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        pusherServo.setPosition(PUSHER_RETRACTED);
        pusherExtended = false;

        // Set initial PIDF coefficients
        setPIDFCoefficients();
    }
    
    /**
     * Update PIDF coefficients on the motor
     */
    private void setPIDFCoefficients() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    
    /**
     * Handle mode switching with X button
     */
    private void handleModeSwitch() {
        boolean xButton = gamepad1.x;
        
        if (xButton && !lastXButton) {
            // Cycle through modes
            switch (currentMode) {
                case MANUAL:
                    currentMode = ControlMode.MENU;
                    menuSelection = 0;
                    break;
                case MENU:
                    currentMode = ControlMode.AUTOMATIC;
                    autoDistanceIndex = 0;
                    autoPowerIndex = 0;
                    autoPaused = false;
                    autoModeTimer.reset();
                    break;
                case AUTOMATIC:
                    currentMode = ControlMode.MANUAL;
                    break;
            }
        }
        
        lastXButton = xButton;
    }
    
    /**
     * Handle PIDF coefficient tuning with gamepad
     */
    private void handlePIDFTuning() {
        boolean yButton = gamepad1.y;
        boolean bButton = gamepad1.b;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        
        // Y + D-pad: Adjust kP
        if (yButton && dpadUp && !lastDpadUp) {
            kP += KP_INCREMENT;
            setPIDFCoefficients();
        }
        if (yButton && dpadDown && !lastDpadDown) {
            kP = Math.max(0, kP - KP_INCREMENT);
            setPIDFCoefficients();
        }
        
        // Y + Bumpers: Adjust kI
        if (yButton && rightBumper && !lastRightBumper) {
            kI += KI_INCREMENT;
            setPIDFCoefficients();
        }
        if (yButton && leftBumper && !lastLeftBumper) {
            kI = Math.max(0, kI - KI_INCREMENT);
            setPIDFCoefficients();
        }
        
        // B + D-pad: Adjust kD (only if not in automatic mode or paused)
        if (bButton && dpadUp && !lastDpadUp && (currentMode != ControlMode.AUTOMATIC || autoPaused)) {
            kD += KD_INCREMENT;
            setPIDFCoefficients();
        }
        if (bButton && dpadDown && !lastDpadDown && (currentMode != ControlMode.AUTOMATIC || autoPaused)) {
            kD = Math.max(0, kD - KD_INCREMENT);
            setPIDFCoefficients();
        }
        
        // B + Bumpers: Adjust kF (only if not in automatic mode or paused)
        if (bButton && rightBumper && !lastRightBumper && (currentMode != ControlMode.AUTOMATIC || autoPaused)) {
            kF += KF_INCREMENT;
            setPIDFCoefficients();
        }
        if (bButton && leftBumper && !lastLeftBumper && (currentMode != ControlMode.AUTOMATIC || autoPaused)) {
            kF = Math.max(0, kF - KF_INCREMENT);
            setPIDFCoefficients();
        }
    }
    
    /**
     * Handle manual control mode
     */
    private void handleManualMode() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean yButton = gamepad1.y;
        boolean bButton = gamepad1.b;
        
        // D-pad Up/Down: Cycle through distances (only if not tuning PIDF)
        if (!yButton && !bButton) {
            if (dpadUp && !lastDpadUp) {
                currentDistanceIndex = (currentDistanceIndex + 1) % DISTANCE_PRESETS.length;
            }
            if (dpadDown && !lastDpadDown) {
                currentDistanceIndex = (currentDistanceIndex - 1 + DISTANCE_PRESETS.length) % DISTANCE_PRESETS.length;
            }
        }
        
        // Bumpers: Adjust power level (only if not tuning PIDF)
        if (!yButton && !bButton) {
            if (rightBumper && !lastRightBumper) {
                currentPowerIndex = Math.min(POWER_LEVELS.length - 1, currentPowerIndex + 1);
            }
            if (leftBumper && !lastLeftBumper) {
                currentPowerIndex = Math.max(0, currentPowerIndex - 1);
            }
        }
        
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
    }
    
    /**
     * Handle menu-based selection mode
     */
    private void handleMenuMode() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean aButton = gamepad1.a;
        boolean yButton = gamepad1.y;
        boolean bButton = gamepad1.b;
        
        // D-pad Up/Down: Navigate menu (only if not tuning PIDF)
        if (!yButton && !bButton) {
            if (dpadUp && !lastDpadUp) {
                menuSelection = (menuSelection - 1 + MENU_ITEMS) % MENU_ITEMS;
            }
            if (dpadDown && !lastDpadDown) {
                menuSelection = (menuSelection + 1) % MENU_ITEMS;
            }
        }
        
        // A button: Select menu item
        if (aButton && !lastAButton) {
            if (menuSelection == 0) {
                // Cycle distance preset
                currentDistanceIndex = (currentDistanceIndex + 1) % DISTANCE_PRESETS.length;
            } else if (menuSelection == 1) {
                // Cycle power level
                currentPowerIndex = (currentPowerIndex + 1) % POWER_LEVELS.length;
            }
        }
        
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastAButton = aButton;
    }
    
    /**
     * Handle automatic cycling mode
     */
    private void handleAutomaticMode() {
        boolean aButton = gamepad1.a;
        boolean bButton = gamepad1.b;
        boolean yButton = gamepad1.y;
        
        // B button: Pause/Resume automatic cycling (only if not tuning with B)
        if (bButton && !lastBButton && !yButton) {
            if (!gamepad1.dpad_up && !gamepad1.dpad_down && 
                !gamepad1.left_bumper && !gamepad1.right_bumper) {
                autoPaused = !autoPaused;
                if (!autoPaused) {
                    autoModeTimer.reset();
                }
            }
        }
        
        // A button: Manually advance to next combination
        if (aButton && !lastAButton) {
            advanceAutoCombination();
            autoModeTimer.reset();
            autoAdvancePressed = true;
        } else if (!aButton) {
            autoAdvancePressed = false;
        }
        
        // Automatic advancement
        if (!autoPaused && !autoAdvancePressed && autoModeTimer.seconds() >= AUTO_ADVANCE_DELAY) {
            advanceAutoCombination();
            autoModeTimer.reset();
        }
        
        // Update current settings to match auto mode
        currentDistanceIndex = autoDistanceIndex;
        currentPowerIndex = autoPowerIndex;
        
        lastAButton = aButton;
        lastBButton = bButton;
    }
    
    /**
     * Advance to next distance-power combination in automatic mode
     */
    private void advanceAutoCombination() {
        autoPowerIndex++;
        
        if (autoPowerIndex >= POWER_LEVELS.length) {
            autoPowerIndex = 0;
            autoDistanceIndex++;
            
            if (autoDistanceIndex >= DISTANCE_PRESETS.length) {
                autoDistanceIndex = 0;
                // Completed full cycle
            }
        }
    }
    
    /**
     * Update motor velocity based on current power setting
     */
    private void updateMotor() {
        double powerLevel = POWER_LEVELS[currentPowerIndex];
        double targetVelocity = powerLevel * TICKS_PER_SECOND_AT_MAX_RPM;

        if (powerLevel > 0) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }
    }

    /**
     * Handle pusher servo control
     * Right trigger: Push (extend servo)
     * Release trigger: Retract servo
     */
    private void handlePusher() {
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

        if (rightTriggerPressed && !lastRightTrigger) {
            // Trigger pressed - extend pusher
            pusherServo.setPosition(PUSHER_EXTENDED);
            pusherExtended = true;
        } else if (!rightTriggerPressed && lastRightTrigger) {
            // Trigger released - retract pusher
            pusherServo.setPosition(PUSHER_RETRACTED);
            pusherExtended = false;
        }

        lastRightTrigger = rightTriggerPressed;
    }
    
    /**
     * Display comprehensive telemetry data
     */
    private void displayTelemetry() {
        // Control mode header
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addData("║ CONTROL MODE", currentMode.toString());
        telemetry.addLine("═══════════════════════════════════════");
        
        // Mode-specific information
        switch (currentMode) {
            case MANUAL:
                telemetry.addLine("\nControls:");
                telemetry.addLine("  D-pad Up/Down: Change Distance");
                telemetry.addLine("  Bumpers: Adjust Power Level");
                telemetry.addLine("  X: Switch to Menu Mode");
                break;
            case MENU:
                telemetry.addLine("\nMenu Selection:");
                telemetry.addData("  " + (menuSelection == 0 ? "► " : "  ") + "Distance", 
                    DISTANCE_PRESETS[currentDistanceIndex] + " inches");
                telemetry.addData("  " + (menuSelection == 1 ? "► " : "  ") + "Power", 
                    String.format("%.0f%%", POWER_LEVELS[currentPowerIndex] * 100));
                telemetry.addLine("\nControls:");
                telemetry.addLine("  D-pad Up/Down: Navigate");
                telemetry.addLine("  A: Select Item");
                telemetry.addLine("  X: Switch to Auto Mode");
                break;
            case AUTOMATIC:
                telemetry.addData("\nAuto Status", autoPaused ? "PAUSED" : "RUNNING");
                telemetry.addData("Progress", 
                    String.format("%d/%d combinations", 
                    autoDistanceIndex * POWER_LEVELS.length + autoPowerIndex + 1,
                    DISTANCE_PRESETS.length * POWER_LEVELS.length));
                if (!autoPaused) {
                    telemetry.addData("Next advance in", 
                        String.format("%.1fs", AUTO_ADVANCE_DELAY - autoModeTimer.seconds()));
                }
                telemetry.addLine("\nControls:");
                telemetry.addLine("  A: Advance Now");
                telemetry.addLine("  B: Pause/Resume");
                telemetry.addLine("  X: Switch to Manual Mode");
                break;
        }
        
        // Current test parameters
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("CURRENT TEST PARAMETERS");
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addData("Distance Preset", DISTANCE_PRESETS[currentDistanceIndex] + " inches");
        telemetry.addData("Power Level", String.format("%.0f%%", POWER_LEVELS[currentPowerIndex] * 100));
        
        // Motor performance data
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("MOTOR PERFORMANCE");
        telemetry.addLine("───────────────────────────────────────");
        
        double currentVelocity = shooter.getVelocity();
        double targetVelocity = POWER_LEVELS[currentPowerIndex] * TICKS_PER_SECOND_AT_MAX_RPM;
        double currentRPM = (currentVelocity / MOTOR_TICKS_PER_REV) * 60.0;
        double targetRPM = (targetVelocity / MOTOR_TICKS_PER_REV) * 60.0;
        double velocityError = targetVelocity - currentVelocity;
        double errorPercent = targetVelocity > 0 ? (velocityError / targetVelocity) * 100 : 0;
        
        telemetry.addData("Current Velocity", String.format("%.0f ticks/sec", currentVelocity));
        telemetry.addData("Target Velocity", String.format("%.0f ticks/sec", targetVelocity));
        telemetry.addData("Current RPM", String.format("%.0f RPM", currentRPM));
        telemetry.addData("Target RPM", String.format("%.0f RPM", targetRPM));
        telemetry.addData("Velocity Error", String.format("%.0f ticks/sec (%.1f%%)", velocityError, errorPercent));
        telemetry.addData("Motor Power", String.format("%.2f", shooter.getPower()));

        // Pusher status
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("PUSHER SERVO");
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addData("Pusher State", pusherExtended ? "EXTENDED" : "RETRACTED");
        telemetry.addData("Pusher Position", String.format("%.2f", pusherServo.getPosition()));
        telemetry.addLine("  Right Trigger: Push Sample");

        // PIDF coefficients
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("PIDF COEFFICIENTS");
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addData("kP", String.format("%.2f  (Y + D-pad)", kP));
        telemetry.addData("kI", String.format("%.3f  (Y + Bumpers)", kI));
        telemetry.addData("kD", String.format("%.2f  (B + D-pad)", kD));
        telemetry.addData("kF", String.format("%.2f  (B + Bumpers)", kF));
        
        // System information
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addData("Runtime", String.format("%.1f seconds", runtime.seconds()));
        telemetry.addData("Motor", "goBILDA 5203 (6000 RPM)");
    }
}
