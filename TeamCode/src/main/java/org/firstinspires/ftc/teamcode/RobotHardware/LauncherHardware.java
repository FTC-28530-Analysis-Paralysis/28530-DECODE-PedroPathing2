package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LauncherHardware {

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private IndicatorLightHardware indicatorLight = null;

    public static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);

    // --- LAUNCHER SPEED INTERPOLATION CONSTANTS --- (NOW QUADRATIC)
    private static final Pose   CLOSE_SHOT_POSE   = new Pose(108, 108, 0);
    private static final double CLOSE_SHOT_RPM    = 2400;
    private static final Pose   MID_SHOT_POSE     = new Pose(72, 72, Math.toRadians(45)); // New mid-point
    private static final double MID_SHOT_RPM      = 2500.0; // RPM at the new mid-point
    private static final Pose   FAR_SHOT_POSE     = new Pose(90, 12, 0);
    private static final double FAR_SHOT_RPM      = 2760.0;

    // Cached distances for interpolation, calculated on first use to be safe.
    private static double closeShotDistance = -1;
    private static double midShotDistance = -1;
    private static double farShotDistance = -1;

    public static final double RPM_TOLERANCE = 25;
    public static final double TICKS_PER_REV = 28;
    private double targetRPM = 0;
    private boolean isSpinning = false;

    public void init(HardwareMap hardwareMap, IndicatorLightHardware indicatorLight) {
        this.indicatorLight = indicatorLight;
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    public void update(Pose robotPose) {
        if (isSpinning) {
            if (robotPose == null) { // Safety check
                setLaunchSpeed(CLOSE_SHOT_RPM); // Default to close shot RPM if localization is lost
            } else {
                Pose targetGoal = (GameState.alliance == GameState.Alliance.BLUE)
                        ? FieldPosePresets.BLUE_GOAL_TARGET
                        : FieldPosePresets.RED_GOAL_TARGET;

                double distance = Math.hypot(targetGoal.getX() - robotPose.getX(), targetGoal.getY() - robotPose.getY());
                double calculatedSpeed = quadraticInterpolate(distance);
                setLaunchSpeed(calculatedSpeed);
            }

            if (isAtTargetSpeed()) {
                indicatorLight.setGreen();
            } else {
                indicatorLight.setRed();
            }
        } else {
            setLaunchSpeed(0);
        }
        indicatorLight.update();
    }

    public void start() {
        isSpinning = true;
    }

    public void stop() {
        isSpinning = false;
        setLaunchSpeed(0);
        if (indicatorLight != null) {
            indicatorLight.setOff();
        }
    }

    public boolean isSpinning() {
        return isSpinning;
    }

    private double quadraticInterpolate(double currentDistance) {
        // Calculate reference distances on the first call.
        if (closeShotDistance < 0) {
            // The tuning points are on the red side, so we use the red goal as our reference for all.
            closeShotDistance = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - CLOSE_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - CLOSE_SHOT_POSE.getY());
            midShotDistance   = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - MID_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - MID_SHOT_POSE.getY());
            farShotDistance   = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - FAR_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - FAR_SHOT_POSE.getY());
        }

        // Lagrange Polynomial for quadratic interpolation (y = ...)
        double y1 = CLOSE_SHOT_RPM;
        double y2 = MID_SHOT_RPM;
        double y3 = FAR_SHOT_RPM;

        double x1 = closeShotDistance;
        double x2 = midShotDistance;
        double x3 = farShotDistance;
        double x = currentDistance;

        double term1 = y1 * ((x - x2) * (x - x3)) / ((x1 - x2) * (x1 - x3));
        double term2 = y2 * ((x - x1) * (x - x3)) / ((x2 - x1) * (x2 - x3));
        double term3 = y3 * ((x - x1) * (x - x2)) / ((x3 - x1) * (x3 - x2));

        double calculatedSpeed = term1 + term2 + term3;
        return Math.max(0, calculatedSpeed); // Prevent negative speeds
    }

    public void setLaunchSpeed(double targetRPM) {
        this.targetRPM = targetRPM;
        double targetVelocity_TPS = (targetRPM * TICKS_PER_REV) / 60.0;
        rightLauncher.setVelocity(targetVelocity_TPS);
        leftLauncher.setVelocity(targetVelocity_TPS);
    }

    public void reverse() {
        rightLauncher.setVelocity(-1000);
        leftLauncher.setVelocity(-1000);
        if (indicatorLight != null) {
            indicatorLight.setOff();
        }
    }

    public boolean isAtTargetSpeed() {
        double leftError = Math.abs(targetRPM - getLeftFlywheelRPM());
        double rightError = Math.abs(targetRPM - getRightFlywheelRPM());
        return (leftError < RPM_TOLERANCE) && (rightError < RPM_TOLERANCE);
    }

    public double getLeftFlywheelRPM() {
        return (leftLauncher.getVelocity() * 60.0 / TICKS_PER_REV);
    }

    public double getRightFlywheelRPM(){
        return (rightLauncher.getVelocity() * 60.0 / TICKS_PER_REV);
    }

    public double getTargetRPM(){
        return targetRPM;
    }
    public double getLeftFlywheelCurrent() { return leftLauncher.getCurrent(CurrentUnit.AMPS); }
    public double getRightFlywheelCurrent() { return rightLauncher.getCurrent(CurrentUnit.AMPS); }
}
