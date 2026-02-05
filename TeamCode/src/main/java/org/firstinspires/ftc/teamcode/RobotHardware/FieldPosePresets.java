package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * This class contains preset static field positions for the robot.
 * Using a central class for these poses allows for easy tuning and consistency
 * across different OpModes (TeleOp and Autonomous).
 */
public class FieldPosePresets {

    // ========== GOAL TARGETS ==========
    // These are the center points of the goals for aiming calculations.
    public static Pose BLUE_GOAL_TARGET = new Pose(0, 144, Math.toRadians(315));
    public static Pose RED_GOAL_TARGET = new Pose(144, 144, Math.toRadians(45));

    // ========== BLUE ALLIANCE POSES ==========
    public static Pose BLUE_FRONT_START = new Pose(56, 9, Math.toRadians(90));
    public static Pose BLUE_BACK_START = new Pose(33, 135, Math.toRadians(270));

    public static Pose BLUE_SCORE_CLOSE_TO_GOAL = new Pose(60, 84, Math.toRadians(135));
    public static Pose BLUE_SCORE_FAR_FROM_GOAL = new Pose(54, 12, Math.toRadians(115));

    public static Pose BLUE_PICKUP_FRONT_SPIKE = new Pose(40, 36, Math.toRadians(180));
    public static Pose BLUE_PICKUP_MIDDLE_SPIKE = new Pose(40, 60, Math.toRadians(180));
    public static Pose BLUE_PICKUP_BACK_SPIKE = new Pose(40, 84, Math.toRadians(180));

    public static Pose BLUE_AUTO_PARK = new Pose(60, 60, Math.toRadians(180));

    // Added Gate Poses for Blue Alliance
    public static Pose BLUE_GATE_APPROACH = new Pose(20, 72, Math.toRadians(90));
    public static Pose BLUE_GATE_TRIGGER = new Pose(16, 72, Math.toRadians(90));


    // ========== RED ALLIANCE POSES ==========
    public static Pose RED_FRONT_START = new Pose(88, 9, Math.toRadians(90));
    public static Pose RED_BACK_START = new Pose(111, 135, Math.toRadians(270));

    public static Pose RED_SCORE_CLOSE_TO_GOAL = new Pose(84, 84, Math.toRadians(45));
    public static Pose RED_SCORE_FAR_FROM_GOAL = new Pose(90, 12, Math.toRadians(63.5));

    public static Pose RED_PICKUP_FRONT_SPIKE = new Pose(104, 36, Math.toRadians(0));
    public static Pose RED_PICKUP_MIDDLE_SPIKE = new Pose(104, 60, Math.toRadians(0));
    public static Pose RED_PICKUP_BACK_SPIKE = new Pose(104, 84, Math.toRadians(0));

    public static Pose RED_AUTO_PARK = new Pose(84, 60, Math.toRadians(0));

    // Added Gate Poses for Red Alliance
    public static Pose RED_GATE_APPROACH = new Pose(124, 72, Math.toRadians(90));
    public static Pose RED_GATE_TRIGGER = new Pose(128, 72, Math.toRadians(90));

    // ========== SHARED POSES ==========
    /** The end game parking locations. */
    public static Pose BLUE_BASE = new Pose(104.5, 33, Math.toRadians(90));
    public static Pose RED_BASE = new Pose(39.5, 33, Math.toRadians(90));

}
