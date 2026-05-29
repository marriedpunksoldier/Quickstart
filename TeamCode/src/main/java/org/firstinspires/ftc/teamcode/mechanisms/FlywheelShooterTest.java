package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

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
@TeleOp(name = "Flywheel Shooter Test", group = "Test")
@Disabled
public class FlywheelShooterTest extends LinearOpMode {
    
    // Hardware
    private DcMotorEx shooter;
    private DcMotor frontIntake;

    // Intake power
    private static final double INTAKE_POWER = 1.0;

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

            // Handle intake
            handleIntake();

            // Display telemetry
            displayTelemetry();
            
            telemetry.update();
        }
        
        // Stop all when OpMode ends
        shooter.setPower(0);
        frontIntake.setPower(0);
    }
    
    /**
     * Initialize hardware components
     */
    private void initializeHardware() {
        // Configure flywheel motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure intake motor
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
     * Update motor output using KSV controller
     */
    private void updateMotor() {
        double powerLevel = POWER_LEVELS[currentPowerIndex];
        if (powerLevel <= 0) {
            shooter.setPower(0);
            return;
        }
        double targetRPM = powerLevel * ShooterConfig.MOTOR_FREE_SPEED_RPM;
        double actualRPM = (shooter.getVelocity() / ShooterConfig.TICKS_PER_REV) * 60.0;
        double feedForward = ShooterConfig.KV_INITIAL * targetRPM + ShooterConfig.KS_INITIAL;
        double pTerm = ShooterConfig.KP_INITIAL * (targetRPM - actualRPM);
        double output = Math.max(0.0, Math.min(1.0, feedForward + pTerm));
        shooter.setPower(output);
    }

    /**
     * Handle intake motor control (gamepad2)
     * Left Bumper: Forward
     * Left Trigger: Reverse
     */
    private void handleIntake() {
        if (gamepad2.left_bumper) {
            frontIntake.setPower(INTAKE_POWER);
        } else if (gamepad2.left_trigger > 0.5) {
            frontIntake.setPower(-INTAKE_POWER);
        } else {
            frontIntake.setPower(0);
        }
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
                    String.format(Locale.US, "%.0f%%", POWER_LEVELS[currentPowerIndex] * 100));
                telemetry.addLine("\nControls:");
                telemetry.addLine("  D-pad Up/Down: Navigate");
                telemetry.addLine("  A: Select Item");
                telemetry.addLine("  X: Switch to Auto Mode");
                break;
            case AUTOMATIC:
                telemetry.addData("\nAuto Status", autoPaused ? "PAUSED" : "RUNNING");
                telemetry.addData("Progress", 
                    String.format(Locale.US, "%d/%d combinations", 
                    autoDistanceIndex * POWER_LEVELS.length + autoPowerIndex + 1,
                    DISTANCE_PRESETS.length * POWER_LEVELS.length));
                if (!autoPaused) {
                    telemetry.addData("Next advance in", 
                        String.format(Locale.US, "%.1fs", AUTO_ADVANCE_DELAY - autoModeTimer.seconds()));
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
        telemetry.addData("Target Power Level", String.format(Locale.US, "%.0f%%", POWER_LEVELS[currentPowerIndex] * 100));
        
        // Motor performance data
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("MOTOR PERFORMANCE");
        telemetry.addLine("───────────────────────────────────────");
        
        double actualRPM = (shooter.getVelocity() / ShooterConfig.TICKS_PER_REV) * 60.0;
        double targetRPM = POWER_LEVELS[currentPowerIndex] * ShooterConfig.MOTOR_FREE_SPEED_RPM;
        double rpmError = targetRPM - actualRPM;

        telemetry.addData("Actual RPM", String.format(Locale.US, "%.0f RPM", actualRPM));
        telemetry.addData("Target RPM", String.format(Locale.US, "%.0f RPM", targetRPM));
        telemetry.addData("RPM Error", String.format(Locale.US, "%.0f RPM", rpmError));
        telemetry.addData("Motor Output", String.format(Locale.US, "%.3f", shooter.getPower()));

        // Mechanisms status
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("MECHANISMS");
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "FORWARD" :
                (frontIntake.getPower() < 0 ? "REVERSE" : "OFF"));
        telemetry.addLine("  GP2 Left Bumper/Trigger: Intake Fwd/Rev");

        // KSV constants
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addLine("KSV CONSTANTS");
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addData("KS", String.format(Locale.US, "%.4f", ShooterConfig.KS_INITIAL));
        telemetry.addData("KV", String.format(Locale.US, "%.6f", ShooterConfig.KV_INITIAL));
        telemetry.addData("KP", String.format(Locale.US, "%.4f", ShooterConfig.KP_INITIAL));

        // System information
        telemetry.addLine("\n───────────────────────────────────────");
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f seconds", runtime.seconds()));
        telemetry.addData("Motor", "goBILDA 5203 (6000 RPM)");
    }
}
