# Zippy Test Hardware Configuration - Complete Setup Guide

## Overview
This guide will walk you through deploying and activating your robot's hardware configuration file for the 2025-2026 FTC season.

---

## What You've Received

**Configuration File:** `zippy_test.xml`

**Configured Components:**
- ✅ 4× goBILDA 5202 Series Motors (drive system)
- ✅ 2× goBILDA Continuous Rotation Servos
- ✅ Built-in Control Hub IMU
- ⚠️ goBILDA Pinpoint Odometry Computer (requires manual configuration)

---

## Part 1: Preparing the Configuration File

### Step 1: Update Your Control Hub Serial Number

**If using a Control Hub (recommended):**
The file is already set correctly with:
```xml
serialNumber="(embedded)"
parentModuleAddress="2"
```
**No changes needed!**

**If using an Expansion Hub with an Android phone:**
1. Power on your Expansion Hub
2. Connect your Robot Controller phone
3. Open the Robot Controller app
4. Go to Settings → Configure Robot → Scan
5. Note the serial number (format: DQ123456 or similar)
6. Open `zippy_test.xml` in Android Studio
7. Replace `serialNumber="(embedded)"` with your actual serial number
8. Change `name="Control Hub Portal"` to `name="Expansion Hub Portal 1"`

### Step 2: Place the File in Your Project

In Android Studio:
1. Navigate to: `TeamCode/src/main/res/xml/`
2. If the `xml` folder doesn't exist:
   - Right-click on `res`
   - Select New → Directory
   - Name it `xml`
3. Copy `zippy_test.xml` into this directory

**File location should be:**
```
TeamCode/
  └── src/
      └── main/
          └── res/
              └── xml/
                  └── zippy_test.xml
```

---

## Part 2: Deploying the Configuration

### Step 3: Deploy to Robot Controller

1. Connect your Robot Controller to your computer via USB or WiFi
2. In Android Studio, click **Run → Team Code** (or press Shift+F10)
3. Wait for the build to complete and the app to install
4. The Robot Controller app will restart automatically

### Step 4: Activate the Configuration

On your Driver Station:
1. Tap the **three vertical dots (⋮)** in the upper right corner
2. Select **"Configure Robot"**
3. You should see **"Zippy Test"** in the list of configurations
4. Tap on **"Zippy Test"**
5. Tap **"Activate"** at the bottom of the screen
6. Press the **back button** to return to the main screen

**Verification:** The active configuration name should now display in the upper right of the Robot Controller screen.

---

## Part 3: Adding the Pinpoint Odometry Computer

### Why Manual Configuration is Required

The goBILDA Pinpoint Odometry Computer uses a custom I2C driver that must be registered through the Robot Controller's device detection system. This ensures proper initialization of its sensor fusion algorithms and communication protocol.

### Step 5: Physical Setup

**Before configuring in software:**

1. **Connect the Pinpoint to I2C Port 0:**
   - Use a 4-pin JST PH I2C cable
   - Connect to the port labeled "0" on your Control Hub
   - The Pinpoint should be mounted with the sticker and ports facing up

2. **Connect Two Odometry Pods to the Pinpoint:**
   - **X Pod (Forward):** Tracks forward/backward movement → Connect to X port
   - **Y Pod (Strafe):** Tracks left/right movement → Connect to Y port
   - Use the cables provided with the goBILDA odometry pods

3. **Power On:**
   - Turn on your Control Hub
   - The Pinpoint LED should illuminate
   - Wait for the LED to turn **GREEN** (indicates ready state)

**LED Status Guide:**
- 🔴 RED: Calibrating or not ready
- 🟢 GREEN: Ready and operational
- 🟣 PURPLE: No pods detected
- 🔵 BLUE: X pod not detected
- 🟠 ORANGE: Y pod not detected

### Step 6: Configure in Robot Controller

On your Driver Station:

1. **Navigate to Configuration:**
   - Tap the three dots (⋮) → "Configure Robot"
   - Tap "Zippy Test" → "Edit"

2. **Access Control Hub:**
   - Tap "Control Hub Portal" (or "Expansion Hub Portal 1")
   - Tap "Control Hub" (or "Expansion Hub 1")

3. **Open I2C Bus 0:**
   - Tap "I2C Bus 0"
   - You should see "imu" already configured here
   - Tap the **"Add"** button at the bottom

4. **Select the Pinpoint Device:**
   - Scroll through the device list
   - Look for **"goBILDA Pinpoint Odometry Computer"**
   - If you don't see it, you may need to update your FTC SDK
   - Tap to select it

5. **Name the Device:**
   - In the "Name" field, enter: **pinpoint**
   - (You can use a different name, but "pinpoint" is the standard)
   - This name will be used in your code

6. **Save Everything:**
   - Tap **"Done"** (returns to I2C Bus 0 screen)
   - Tap **"Done"** (returns to Control Hub screen)
   - Tap **"Done"** (returns to configuration screen)
   - Tap **"Save"** at the top

7. **Verify:**
   - The configuration should now show as saved
   - Press back to return to the main Robot Controller screen

---

## Part 4: Testing Your Configuration

### Step 7: Test Drive Motors

Create a simple test OpMode to verify each motor:

```java
@TeleOp(name="Motor Test", group="Test")
public class MotorTest extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    
    @Override
    public void runOpMode() {
        // Initialize motors using the names from your config file
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftRear = hardwareMap.dcMotor.get("LeftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        // Test each motor at 50% power for 2 seconds
        testMotor(leftFront, "Left Front");
        testMotor(leftRear, "Left Rear");
        testMotor(rightFront, "Right Front");
        testMotor(rightRear, "Right Rear");
    }
    
    private void testMotor(DcMotor motor, String name) {
        telemetry.addData("Testing", name);
        telemetry.update();
        motor.setPower(0.5);
        sleep(2000);
        motor.setPower(0);
        sleep(1000);
    }
}
```

**What to check:**
- Each wheel should rotate forward when tested
- If a wheel rotates backward, you'll need to reverse it in code using `motor.setDirection(DcMotorSimple.Direction.REVERSE)`

### Step 8: Test Servos

```java
@TeleOp(name="Servo Test", group="Test")
public class ServoTest extends LinearOpMode {
    private CRServo intakeServo, indexerServo;
    
    @Override
    public void runOpMode() {
        intakeServo = hardwareMap.crservo.get("intakeServo");
        indexerServo = hardwareMap.crservo.get("indexerServo");
        
        waitForStart();
        
        // Test intake servo
        telemetry.addData("Testing", "Intake Servo");
        telemetry.update();
        intakeServo.setPower(1.0);
        sleep(2000);
        intakeServo.setPower(0);
        sleep(1000);
        
        // Test indexer servo
        telemetry.addData("Testing", "Indexer Servo");
        telemetry.update();
        indexerServo.setPower(1.0);
        sleep(2000);
        indexerServo.setPower(0);
    }
}
```

### Step 9: Test Pinpoint Odometry

```java
@TeleOp(name="Pinpoint Test", group="Test")
public class PinpointTest extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;
    
    @Override
    public void runOpMode() {
        // Initialize Pinpoint using the name from your config
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // Configure for goBILDA 4-bar odometry pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        // Set encoder directions (adjust these based on your setup)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        
        // Reset position and IMU
        pinpoint.resetPosAndIMU();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update position data
            pinpoint.update();
            
            // Get current position
            Pose2D pos = pinpoint.getPosition();
            
            // Display telemetry
            telemetry.addData("X Position", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", Math.toDegrees(pos.getHeading(AngleUnit.RADIANS)));
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();
            
            sleep(50);
        }
    }
}
```

**What to check:**
- LED should be GREEN
- When you push the robot forward, X should increase
- When you push the robot left, Y should increase
- When you rotate the robot counterclockwise, heading should increase
- If any direction is wrong, adjust encoder directions in your code

---

## Common Issues and Solutions

### Issue: Configuration file doesn't appear in the list

**Solution:**
- Verify the file is in `TeamCode/src/main/res/xml/`
- Ensure filename is all lowercase with no spaces
- Rebuild and redeploy: Run → Team Code
- Restart the Robot Controller app

### Issue: "goBILDA Pinpoint Odometry Computer" not in device list

**Possible causes:**
1. **Outdated SDK:** Update to FTC SDK v10.1 or later
2. **Missing Driver:** The Pinpoint driver might not be included. Download from:
   - https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint
3. **Connection Issue:** Verify I2C cable is fully inserted

### Issue: Pinpoint LED is not GREEN

**LED Colors and Solutions:**
- 🔴 **RED:** Still calibrating, wait 5-10 seconds
- 🟣 **PURPLE:** No pods detected → Check pod connections
- 🔵 **BLUE:** X pod missing → Check X port connection
- 🟠 **ORANGE:** Y pod missing → Check Y port connection

### Issue: Motors configured but not responding

**Checklist:**
1. Verify motor names in code match XML exactly (case-sensitive)
2. Check power connections to Control Hub
3. Ensure battery is charged (>12V)
4. Test with different motor port if suspected hardware issue

### Issue: Pinpoint position drifts when stationary

**Solution:**
- The robot must be completely still during calibration
- Call `pinpoint.resetPosAndIMU()` while the robot is stationary
- Ensure the Pinpoint is mounted securely (vibration causes drift)

---

## Configuration File Reference

### Device Names (as configured in XML):

**Motors:**
- `LeftFront` → Port 0
- `LeftRear` → Port 1  
- `rightFront` → Port 2
- `rightRear` → Port 3

**Servos:**
- `intakeServo` → Port 0
- `indexerServo` → Port 1

**Sensors:**
- `imu` → Built-in IMU (I2C Bus 0)
- `pinpoint` → Pinpoint Odometry (I2C Bus 0, manually configured)

### Using Device Names in Code:

```java
// Motors
DcMotor leftFront = hardwareMap.dcMotor.get("LeftFront");

// Servos  
CRServo intakeServo = hardwareMap.crservo.get("intakeServo");

// Pinpoint
GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

// Built-in IMU
IMU imu = hardwareMap.get(IMU.class, "imu");
```

---

## Additional Resources

**FTC Official Documentation:**
- SDK Documentation: https://ftc-docs.firstinspires.org/
- Hardware Configuration: https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Configuring-Your-Hardware

**goBILDA Resources:**
- Pinpoint User Guide: https://www.gobilda.com/content/user_manuals/3110-0002-0001_user-guide.pdf
- Pinpoint GitHub Driver: https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint

**Motor Specifications:**
- goBILDA 5202 Series: https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/

---

## Need Help?

If you encounter issues not covered in this guide:

1. **Check the FTC Community Forum:**
   - https://ftc-community.firstinspires.org/

2. **Review FTC Discord Channels:**
   - Many experienced teams provide real-time help

3. **Consult Your Team Mentor:**
   - They may have experience with similar configurations

4. **Contact goBILDA Support:**
   - For Pinpoint-specific issues: support@gobilda.com

---

**Good luck with your robot configuration! 🤖**

Generated for FTC Team using the 2025-2026 SDK
Configuration Name: Zippy Test
