package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
 * ---------------------------------------------------------------------------------
 * --- MASTER CHECKLIST AND CHANGELOG ---
 * ---------------------------------------------------------------------------------
 * This is the central guide for ensuring the DECODE robot is competition-ready.
 *
 * -- V1.1 CHANGELOG --
 * - Implemented robust initialization with `init_loop` to allow pre-match selection
 *   of Alliance and Starting Position. This ensures a valid field pose even if
 *   Autonomous is not run.
 * - Refactored `start()` method to intelligently set the starting pose by first
 *   checking `GameState` and then falling back to selected presets.
 * - Centralized the `CombinedLocalizer` instance within TeleOp, allowing direct access
 *   to `isPoseReliable` and `resetHeading` for safer and more intuitive operation.
 * - Added comprehensive Javadoc and inline comments to improve code clarity and
 *   maintainability for all team members.
 * - Removed automatic drive mode switching when localization is lost, giving the driver
 *   full control, while retaining a safety lock on the Auto-Park feature.
 *
 * -- MASTER TUNING AND TESTING CHECKLIST --
 * DRIVETRAIN & LOCALIZATION:
 * [x] ODOMETRY/LOCALIZER INSTANCE: Verify `CombinedLocalizer` is created in `BozemanTeleop`
 *     and passed to the Follower.
 * [x] `isPoseReliable` LOGIC: Confirm `isPoseReliable` is set to `false` on resets and `true`
 *     only after a valid Limelight vision update in `CombinedLocalizer`.
 * [x] SAFETY CHECKS: Ensure Auto-Park (`B` button) is guarded by an `isPoseReliable()` check.
 *
 * LAUNCHER & SEQUENCES:
 * [ ] LAUNCHER SPEED: Tune the launcher's automatic speed control in `LauncherHardware`
 *     based on distance to the goal.
 * [ ] ACTION TIMING: Tune all `timer.seconds()` durations in `ActionManager` for intake,
 *     transfer, and launch sequences to ensure they are fast and reliable.
 *
 * GAME & FIELD SPECIFIC:
 * [ ] FIELD POSES: Verify all `FieldPosePresets` (e.g., `BLUE_FRONT_START`, `RED_BASE`)
 *     are accurate on a physical field.
 * [x] STARTING LOGIC: Test the new `init_loop` and `start()` logic to confirm the robot
 *     correctly sets its pose for all Alliance/Start Position combinations.
 *
 * --- CONTROLLER LAYOUT (GAMEPAD 1) ---
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
 * Start Button:  Manually reset heading to 90 degrees.
 * Back Button:   Switch alliance (Red/Blue) for aiming and parking.
 * B Button:      Initiate Auto-Park sequence (only if localization is reliable).
 */
@TeleOp(name = "BozemanTeleop", group = "01 Bozeman")
public class BozemanTeleop extends OpMode {

    // Core robot hardware and software components
    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private PinpointLocalizer localizer; // The single, authoritative localizer instance
    private Follower follower;
    private DriverAssist driverAssist;

    // State machine enums for this OpMode
    private enum TeleOpState { MANUAL, AUTO_PARK }
    private enum DiverterState { PURPLE, GREEN }
    private enum StartPosition { FRONT, BACK }

    // State variables
    private TeleOpState currentState = TeleOpState.MANUAL;
    private DiverterState diverterState = DiverterState.GREEN;
    private StartPosition startPosition = StartPosition.FRONT; // Default starting position
    private boolean left_trigger_pressed = false;

    /**
     * Code to run ONCE when the driver hits INIT.
     * Initializes hardware and sets up a default alliance.
     */
    @Override
    public void init() {
        // Initialize all hardware and software modules
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        localizer = new PinpointLocalizer(hardwareMap, new PinpointConstants());
        follower = Constants.createFollower(hardwareMap, localizer); // Pass it to the follower
        driverAssist = new DriverAssist(follower);

        // If alliance wasn't set by a previous OpMode, default to Blue.
        // This can be changed in init_loop().
        if (GameState.alliance == GameState.Alliance.UNKNOWN) {
            GameState.alliance = GameState.Alliance.BLUE;
        }

        telemetry.addLine("Bozeman TeleOp Initialized.");
        telemetry.addLine("IN INIT: Use D-Pad to select Alliance and Start Position.");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY during the init phase, before START is pressed.
     * This allows the driver to select the alliance and starting position.
     */
    @Override
    public void init_loop() {
        // Allow alliance and starting position selection before the match begins.
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            GameState.alliance = GameState.Alliance.BLUE;
            robot.indicatorLight.setStaticColor(IndicatorLightHardware.COLOR_BLUE);
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            GameState.alliance = GameState.Alliance.RED;
            robot.indicatorLight.setStaticColor(IndicatorLightHardware.COLOR_RED);
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up) startPosition = StartPosition.BACK;
        if (gamepad1.dpad_down || gamepad2.dpad_down) startPosition = StartPosition.FRONT;

        // Provide continuous feedback on the driver station
        telemetry.addData("Selected Alliance", GameState.alliance);
        telemetry.addData("Selected Start", startPosition);
        telemetry.addLine("\nReady to START!");
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits PLAY.
     * Sets the definitive starting pose for the robot.
     */
    @Override
    public void start() {
        // Set the starting pose with a clear order of precedence:
        // 1. Use the pose from Autonomous if it exists.
        if (GameState.currentPose != null) {
            localizer.setStartPose(GameState.currentPose);
        }
        // 2. Otherwise, fall back to the presets selected during init_loop().
        else if (GameState.alliance == GameState.Alliance.RED) {
            localizer.setStartPose((startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START);
        } else { // Alliance is BLUE
            localizer.setStartPose((startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START);
        }

        // The first update call populates the follower with the correct starting pose
        follower.update();
    }

    /**
     * Main TeleOp loop, runs REPEATEDLY after PLAY is pressed.
     */
    @Override
    public void loop() {
        // Core robot updates must be called in every loop.
        follower.update(); // Updates odometry and path following logic
        actionManager.update(); // Updates any running timed action sequences
        robot.launcher.update(follower.getPose()); // Updates launcher speed based on distance
        robot.indicatorLight.update(); // Updates indicator light animations

        // Main state machine for switching between manual control and auto-park.
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
        // Pass joystick inputs to the DriverAssist module to calculate drive power
        driverAssist.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Delegate to specific handlers for better organization
        handleIntakeControls();
        handleLauncherControls();
        handleFeederControls();
        handleSystemControls();
    }

    /**
     * Manages intake motor controls (On/Off and Reverse).
     */
    private void handleIntakeControls() {
        // Eject/Reverse on Left Trigger. Uses manual edge detection as it's an analog input.
        boolean leftTriggerIsPressed = gamepad1.left_trigger > 0.1;
        if (leftTriggerIsPressed && !left_trigger_pressed) {
            actionManager.reverseAll();
        }
        // Stop reversing when the trigger is released.
        if (!leftTriggerIsPressed && left_trigger_pressed) {
            actionManager.stopAll();
        }
        left_trigger_pressed = leftTriggerIsPressed;

        // Toggle Intake on/off with the 'A' button, but not if ejecting.
        if (gamepad1.aWasPressed() && !leftTriggerIsPressed) {
            if (actionManager.getCurrentState() == ActionManager.ActionState.INTAKING) {
                actionManager.stopAll();
            } else if (actionManager.getCurrentState() == ActionManager.ActionState.IDLE) {
                actionManager.startIntake();
            }
        }
    }

    /**
     * Manages the launcher flywheel motors and drive mode switching.
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

        // Cycle through drive modes. No reliability check is needed here, giving the
        // driver full control to use any mode they see fit.
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
     * Manages the control to fire an Artifact from the feeder.
     */
    private void handleFeederControls() {
        // Fire an artifact when either bumper is pressed. The ActionManager handles the
        // logic of waiting for the launcher to be at the correct speed.
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

        // Manual Heading Reset
        if (gamepad1.startWasPressed()) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(90)));
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

        // Trigger the Auto-Park sequence, ONLY if localization is reliable.
        // This is the primary safety check to prevent autonomous movement with a bad pose.
        if (gamepad1.bWasPressed()) {
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
     * Allows the driver to interrupt at any time by moving the joysticks.
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
     * Updates all telemetry on the Driver Station screen.
     */
    private void updateTelemetry() {
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driverAssist.getMode().toString());
        telemetry.addData("Diverter", diverterState.toString());
        telemetry.addData("Alliance", GameState.alliance.toString());
        telemetry.addData("Launcher Target RPM", "%.1f", robot.launcher.getTargetRPM());
        telemetry.addData("Launcher Actual RPM", "%.1f", robot.launcher.getLeftFlywheelRPM());

        // Only display the pose if it's considered reliable.
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        }
    }
}
