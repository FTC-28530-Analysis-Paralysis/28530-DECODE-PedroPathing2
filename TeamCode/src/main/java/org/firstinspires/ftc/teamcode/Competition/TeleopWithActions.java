package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "ButteTeleop (Actions)", group = "01 Butte")
@Disabled // This OpMode is in development, preserving the @Disabled tag.
public class TeleopWithActions extends OpMode {

    RobotHardwareContainer robot;
    ActionManager actionManager;
    Follower follower;

    // State machine for TeleOp control
    private enum TeleOpState { MANUAL, AUTO_PARK }
    private TeleOpState currentState = TeleOpState.MANUAL;

    // Alliance selection for auto-park
    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.BLUE;

    // Button press state trackers
    private boolean dpad_right_pressed = false;
    private boolean y_pressed = false;
    private boolean b_pressed = false;
    private boolean x_pressed = false;

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        // Initialize the follower with our fused localizer
        follower = Constants.createFollower(hardwareMap, robot.localizer);

        telemetry.addLine("Actions TeleOp Initialized. Ready to drive!");
        telemetry.addLine("Press X to toggle Alliance for Auto-Park.");
        telemetry.addLine("Press B to start Auto-Park.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ALWAYS update the follower and action manager
        // This runs our fused localization and manages mechanism sequences.
        follower.update();
        actionManager.update();

        switch (currentState) {
            case MANUAL:
                // Drive control is now handled by the follower for seamless transitions
                follower.setDrivePowers(new Pose(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

                // Alliance selection for parking
                if (gamepad1.x && !x_pressed) {
                    alliance = (alliance == Alliance.BLUE) ? Alliance.RED : Alliance.BLUE;
                }

                // Auto-park trigger. Note: This repurposes the 'B' button from its 'stopAll' function.
                if (gamepad1.b && !b_pressed) {
                    // Select the correct base pose based on the alliance
                    Pose parkPose = (alliance == Alliance.BLUE) ? FieldPosePresets.BLUE_BASE : FieldPosePresets.RED_BASE;
                    // Get the robot's current, accurate position
                    Pose currentPose = follower.getPose();
                    // Build the path
                    Path parkingPath = new Path(new BezierLine(currentPose, parkPose));
                    parkingPath.setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading());
                    // Command the follower to execute the path
                    follower.followPath(parkingPath);
                    // Switch to auto-park state
                    currentState = TeleOpState.AUTO_PARK;
                }

                // Handle the other manual mechanism controls
                handleManualMechanisms();
                break;

            case AUTO_PARK:
                telemetry.addLine("--- AUTO-PARKING --- ");
                // If the follower is no longer busy, return to manual control
                if (!follower.isBusy()) {
                    currentState = TeleOpState.MANUAL;
                }
                // Allow driver to interrupt parking by moving the sticks
                if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                    follower.breakFollowing();
                    currentState = TeleOpState.MANUAL;
                }
                break;
        }

        updateButtonStates();
        updateTelemetry();
    }

    private void handleManualMechanisms() {
        if (gamepad1.right_trigger > 0.1) {
            actionManager.startIntake();
        } else if (gamepad1.left_trigger > 0.1) {
            actionManager.reverseAll();
        } else if (gamepad1.dpad_right && !dpad_right_pressed) {
            actionManager.startLaunch();
        } else if (gamepad1.y && !y_pressed) {
            // Manual override for transfer
            robot.transfer.run();
        } else if (!actionManager.isBusy()) {
            // If no actions are running, ensure all mechanisms are stopped
            actionManager.stopAll();
        }
    }

    private void updateButtonStates() {
        dpad_right_pressed = gamepad1.dpad_right;
        y_pressed = gamepad1.y;
        b_pressed = gamepad1.b;
        x_pressed = gamepad1.x;
    }

    private void updateTelemetry() {
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Selected Alliance for Park", alliance.toString());
        telemetry.addData("Action Manager State", actionManager.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("X Position", "%.2f", follower.getPose().getX());
        telemetry.addData("Y Position", "%.2f", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        actionManager.stopAll();
        follower.breakFollowing();
    }
}
