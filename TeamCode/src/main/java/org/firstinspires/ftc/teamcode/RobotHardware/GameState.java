package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * A static class to hold game state information that needs to persist between OpModes
 * (e.g., from Autonomous to TeleOp).
 * <p>
 * By declaring variables as `public static volatile`, we ensure that any changes made
 * to them in one OpMode (like setting the `currentPose` at the end of Autonomous) are
 * immediately visible and accessible to the next OpMode (like TeleOp).
 * <p>
 * This is a simple but effective way to pass data without writing to files.
 */
public class GameState {

    /** Represents the team's alliance color. */
    public enum Alliance { BLUE, RED, UNKNOWN }

    // The `volatile` keyword ensures that changes are immediately visible across threads/OpModes.
    public static volatile Alliance alliance = Alliance.UNKNOWN;
    public static volatile Pose currentPose = null;

}
