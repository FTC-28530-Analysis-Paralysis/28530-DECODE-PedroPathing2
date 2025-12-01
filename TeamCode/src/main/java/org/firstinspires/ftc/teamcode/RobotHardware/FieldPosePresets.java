package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * This class holds all the constant pose information for the field.
 * By centralizing a "single source of truth" for field locations, we can easily
 * update coordinates for all autonomous routines at once. It also makes the
 * auto OpModes cleaner and easier to read.
 *
 * The coordinate system is based on the PedroPathing coordinate system.
 *
 */
public class FieldPosePresets {

    // ========== BLUE Alliance Poses ==========
    public static final Pose BLUE_FRONT_START = new Pose(56, 9, Math.toRadians(90));
    public static final Pose BLUE_BACK_START = new Pose(33, 135, Math.toRadians(270));

    public static final Pose BLUE_SCORE_POSE = new Pose(60, 84, Math.toRadians(135));
    public static final Pose BLUE_PICKUP_FRONT_SPIKE = new Pose(40, 36, Math.toRadians(0));
    public static final Pose BLUE_PICKUP_MIDDLE_SPIKE = new Pose(40, 60, Math.toRadians(0));
    public static final Pose BLUE_PICKUP_BACK_SPIKE = new Pose(40, 84, Math.toRadians(0));
    public static final Pose BLUE_AUTO_PARK = new Pose(60, 60, Math.toRadians(180));

    // ========== RED Alliance Poses ==========
    public static final Pose RED_FRONT_START = new Pose(88, 9, Math.toRadians(90));
    public static final Pose RED_BACK_START = new Pose(111, 135, Math.toRadians(270));

    public static final Pose RED_SCORE_POSE = new Pose(84, 84, Math.toRadians(45));
    // The pickup poses for RED are mirrored from the BLUE side.
    // The field is 144 inches wide. X_RED = 144 - X_BLUE.
    // The heading is also mirrored. Heading_RED = 180 - Heading_BLUE (for angles in degrees).
    public static final Pose RED_PICKUP_FRONT_SPIKE = new Pose(104, 36, Math.toRadians(180));
    public static final Pose RED_PICKUP_MIDDLE_SPIKE = new Pose(104, 60, Math.toRadians(180));
    public static final Pose RED_PICKUP_BACK_SPIKE = new Pose(104, 84, Math.toRadians(180));
    public static final Pose RED_AUTO_PARK = new Pose(84, 60, Math.toRadians(0));

    // ========== TeleOp Base Poses ==========
    public static final Pose BLUE_BASE = new Pose(38, 33, Math.toRadians(90));
    public static final Pose RED_BASE = new Pose(106, 33, Math.toRadians(90));
}
