package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.IndicatorLightHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Main TeleOp class for the DECODE robot.
 *
 * This class integrates all hardware and software components to create a feature-rich,
 * driver-controlled experience. It is designed to be robust, with safety features
 * to handle localization failures and a clear separation of concerns between driver
 * input, action sequencing, and hardware control.
 *
 * --- KEY FEATURES ---
 * - Advanced Drive Control:
 *   - ROBOT_CENTRIC: Standard, intuitive control.
 *   - FIELD_CENTRIC: Forward is always downfield.
 *   - TARGET_LOCK: Automatically aims the robot at the correct alliance goal.
 * - Dynamic Launcher Speed:
 *   - The launcher speed is continuously and automatically adjusted based on the
 *     robot's distance to the goal, allowing for accurate shots from anywhere on the field.
 * - Robust Safety System:
 *   - The robot continuously monitors localization reliability. If the pose becomes
 *     unreliable (e.g., from a disconnected camera or bad data), it automatically
 *     reverts to a safe ROBOT_CENTRIC drive mode and disables features that depend
 *     on an accurate field position (like Target Lock and Auto-Park).
 * - Centralized Action Management:
 *   - Complex, timed sequences (like firing an Artifact) are handled by the
 *     ActionManager, keeping this TeleOp class clean and focused on driver input.
 *
 * --- BUTTON MAPPINGS (GAMEPAD 1) ---
 *
 * [Driving]
 * Left Stick:  Translate (forward/backward/strafe)
 * Right Stick: Rotate
 *
 * [Intake & Feeder]
 * A Button:      Toggle Intake On/Off
 * Left Bumper:   Fire one Artifact (if launcher is at speed)
 * Right Bumper:  Fire one Artifact (if launcher is at speed)
 * Left Trigger:  Eject/Reverse all intake and feeder motors
 *
 * [Launcher]
 * Y Button:      Toggle launcher motors On/Off (speed is automatic)
 *
 * [System & Drive Modes]
 * D-Pad Right:   Cycle through drive modes (Robot-Centric -> Field-Centric -> Target-Lock)
 * D-Pad Down:    Toggle Artifact Diverter (for scoring in the correct goal)
 * Start Button:  Manually reset heading and return to ROBOT_CENTRIC drive.
 * Back Button:   Switch alliance (Red/Blue) for aiming and parking.
 * B Button:      Initiate Auto-Park sequence (if localization is reliable).
 *
 */
@TeleOp(name = "BozemanTeleop", group = "01 Bozeman")
public class BozemanTeleop extends OpMode {

    // Core robot hardware and software components
    RobotHardwareContainer robot;
    ActionManager actionManager;
    CombinedLocalizer localizer; // Use CombinedLocalizer directly
    Follower follower;
    DriverAssist driverAssist;

    // State machine enums for this OpMode
    private enum TeleOpState { MANUAL, AUTO_PARK }
    private enum DiverterState { PURPLE, GREEN }

    // State variables for this OpMode
    private TeleOpState currentState = TeleOpState.MANUAL;
    private DiverterState diverterState = DiverterState.GREEN; // Default state

    // Button press state tracker for the trigger (which lacks a wasPressed method)
    private boolean left_trigger_pressed = false;

    /**
     * Code to run ONCE when the driver hits INIT.
     * Initializes all robot hardware, software modules, and sets default states.
     */
    @Override
    public void init() {
        // Initialize all hardware and software modules
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        localizer = new CombinedLocalizer(hardwareMap, telemetry); // Create the localizer here
        follower = Constants.createFollower(hardwareMap, localizer); // Pass it to the follower
        driverAssist = new DriverAssist(follower);

        // Default to Blue alliance if not set by an Autonomous OpMode
        if (GameState.alliance == GameState.Alliance.UNKNOWN) {
            GameState.alliance = GameState.Alliance.BLUE;
        }

        // Display driver instructions on the Driver Station
        telemetry.addLine("TeleOp Initialized. Let's DECODE!");
        telemetry.addLine("Press START to reset heading.");
        telemetry.addLine("Press B for Auto-Park.");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP.
     * This is the main loop of the TeleOp.
     */
    @Override
    public void loop() {
        // Core robot updates. These must be called in every loop.
        follower.update(); // Updates odometry and path following
        actionManager.update(); // Updates any running timed actions
        robot.launcher.update(follower.getPose()); // Updates launcher speed based on distance
        robot.indicatorLight.update(); // Updates indicator light animations

        // Main state machine for TeleOp
        switch (currentState) {
            case MANUAL:
                handleManualControls();
                break;
            case AUTO_PARK:
                handleAutoPark();
                break;
        }

        updateTelemetry();
    }

    /**
     * Handles all manual, driver-controlled actions.
     */
    private void handleManualControls() {
        // Pass joystick inputs to the DriverAssist module
        driverAssist.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // --- Delegate to specific control handlers for organization ---
        handleIntakeControls();
        handleLauncherControls();
        handleFeederControls();
        handleSystemControls();
    }

    /**
     * Manages intake and eject controls.
     */
    private void handleIntakeControls() {
        // Eject/Reverse on Left Trigger. Needs manual edge detection because it's an analog input.
        boolean leftTriggerIsPressed = gamepad1.left_trigger > 0.1;
        if (leftTriggerIsPressed && !left_trigger_pressed) {
            actionManager.reverseAll();
        }
        // Stop reversing when the trigger is released.
        if (!leftTriggerIsPressed && left_trigger_pressed) {
            actionManager.stopAll();
        }
        left_trigger_pressed = leftTriggerIsPressed;

        // Toggle Intake on/off with the 'A' button.
        if (gamepad1.aWasPressed() && !leftTriggerIsPressed) {
            if (actionManager.getCurrentState() == ActionManager.ActionState.INTAKING) {
                actionManager.stopAll();
            } else if (actionManager.getCurrentState() == ActionManager.ActionState.IDLE) {
                actionManager.startIntake();
            }
        }
    }

    /**
     * Manages launcher and drive mode controls.
     */
    private void handleLauncherControls() {
        // Toggle launcher on/off with the 'Y' button.
        if (gamepad1.yWasPressed()) {
            if (robot.launcher.isSpinning()) {
                robot.launcher.stop();
            } else {
                robot.launcher.start();
            }
        }

        // Cycle through drive modes, with no restrictions.
        if (gamepad1.dpadRightWasPressed()) {
            switch (driverAssist.getMode()) {
                case ROBOT_CENTRIC:
                    driverAssist.setMode(DriverAssist.DriveMode.FIELD_CENTRIC);
                    break;
                case FIELD_CENTRIC:
                    driverAssist.setMode(DriverAssist.DriveMode.TARGET_LOCK);
                    break;
                case TARGET_LOCK:
                    driverAssist.setMode(DriverAssist.DriveMode.ROBOT_CENTRIC);
                    break;
            }
        }
    }

    /**
     * Manages the control to fire an Artifact.
     */
    private void handleFeederControls() {
        // Fire an artifact when a bumper is pressed.
        // The ActionManager handles the logic of waiting for the launcher to be at speed.
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            actionManager.fireArtifactWhenReady();
        }
    }

    /**
     * Manages system-level controls like alliance switching, parking, and resets.
     */
    private void handleSystemControls() {
        // Switch alliance color with the 'Back' button.
        if (gamepad1.backWasPressed()) {
            GameState.alliance = (GameState.alliance == GameState.Alliance.BLUE) ? GameState.Alliance.RED : GameState.Alliance.BLUE;
            double blinkColor = (GameState.alliance == GameState.Alliance.BLUE) ? IndicatorLightHardware.COLOR_BLUE : IndicatorLightHardware.COLOR_RED;
            robot.indicatorLight.blink(blinkColor, 2); // Blink to confirm selection
        }

        // Manual Heading and Drive Mode Reset
        if (gamepad1.startWasPressed()) {
            localizer.resetHeading(); // Call resetHeading on the localizer instance
            robot.indicatorLight.blink(IndicatorLightHardware.COLOR_BLUE, 1.5); // Blink to confirm reset
        }

        // Toggle the Artifact diverter with D-Pad Down.
        if (gamepad1.dpadDownWasPressed()) {
            if (diverterState == DiverterState.GREEN) {
                actionManager.setDiverterPurple();
                diverterState = DiverterState.PURPLE;
            } else {
                actionManager.setDiverterGreen();
                diverterState = DiverterState.GREEN;
            }
        }

        // Trigger the Auto-Park sequence with the 'B' button, but only if localization is reliable.
        if (gamepad1.bWasPressed() && localizer.isPoseReliable()) {
            Pose parkPose = (GameState.alliance == GameState.Alliance.BLUE) ? FieldPosePresets.BLUE_BASE : FieldPosePresets.RED_BASE;
            Pose currentPose = follower.getPose();
            Path parkingPath = new Path(new BezierLine(currentPose, parkPose));
            parkingPath.setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading());
            follower.followPath(parkingPath);
            currentState = TeleOpState.AUTO_PARK;
        }
    }

    /**
     * Handles the autonomous parking state, waiting for the path to complete.
     * Allows the driver to interrupt by moving the joysticks.
     */
    private void handleAutoPark() {
        telemetry.addLine("--- AUTO-PARKING ---");
        // When the follower is no longer busy, return to manual control.
        if (!follower.isBusy()) {
            currentState = TeleOpState.MANUAL;
        }
        // Allow the driver to interrupt the path by moving the sticks.
        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
            follower.breakFollowing();
            currentState = TeleOpState.MANUAL;
        }
    }

    /**
     * Updates all the telemetry on the Driver Station screen.
     */
    private void updateTelemetry() {
        if (!localizer.isPoseReliable()) {
            telemetry.addLine("!! LOCALIZATION UNRELIABLE - AUTO-PARK DISABLED !!");
        }
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driverAssist.getMode().toString());
        telemetry.addData("Diverter", diverterState.toString());
        telemetry.addData("Alliance", GameState.alliance.toString());
        telemetry.addData("Launcher Target RPM", "%.1f", robot.launcher.getTargetRPM());
        telemetry.addData("Launcher Actual RPM", "%.1f", robot.launcher.getLeftFlywheelRPM());
        if (localizer.isPoseReliable()) {
            telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        }
    }
}
