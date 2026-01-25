package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

/**
 * A helper class to manage advanced TeleOp driving features like field-centric
 * drive and target-locking headings. This class acts as an intelligent layer between
 * the driver's joystick inputs and the Pedro Pathing Follower, which handles the
 * underlying drive calculations.
 */
public class DriverAssist {

    private final Follower follower;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET_LOCK
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

    // Proportional gain for the Target Lock mode.
    // Tune this for responsive but stable target locking.
    private static final double HEADING_KP = 0.8;

    public DriverAssist(Follower follower) {
        this.follower = follower;
    }

    public void setMode(DriveMode mode) {
        this.currentMode = mode;
    }

    public DriveMode getMode() {
        return currentMode;
    }

    /**
     * Main update loop. Passes joystick inputs to the Follower and lets it handle
     * the driving calculations based on the selected mode.
     */
    public void update(double joyY, double joyX, double joyTurn) {
        Pose robotPose = follower.getPose();

        // If localization is not stable, do not send drive commands.
        if (robotPose == null) {
            return;
        }

        switch (currentMode) {
            case ROBOT_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, true);
                break;

            case FIELD_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, false);
                break;

            case TARGET_LOCK:
                double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(robotPose), robotPose.getHeading());
                double calculatedTurn = HEADING_KP * headingError;
                calculatedTurn = Math.max(-1.0, Math.min(1.0, calculatedTurn));

                follower.setTeleOpDrive(joyY, joyX, calculatedTurn, false);
                break;
        }
    }

    /**
     * Calculates the absolute field heading (in radians) from the robot's current position to the goal.
     */
    public double calculateHeadingToGoal(Pose robotPose)
    {
        Pose targetGoal = (GameState.alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        return Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );
    }
}
