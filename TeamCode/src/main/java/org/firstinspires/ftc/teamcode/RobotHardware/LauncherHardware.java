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

    // --- LAUNCHER SPEED INTERPOLATION CONSTANTS ---
    // These two points define the linear relationship between distance and speed.
    // The formula will extrapolate beyond these points for shots that are closer or farther.
    private static final Pose   CLOSE_SHOT_POSE   = new Pose(75, 81, 0);
    private static final double CLOSE_SHOT_RPM    = 1200.0;
    private static final Pose   FAR_SHOT_POSE     = new Pose(84, 11, 0);
    private static final double FAR_SHOT_RPM      = 1350.0;

    // Cached distances for interpolation, calculated on first use to be safe.
    private static double closeShotDistance = -1;
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
                double calculatedSpeed = interpolate(distance);
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

    private double interpolate(double currentDistance) {
        // Calculate the reference distances on the first call to be safe.
        if (closeShotDistance < 0) {
            // The tuning points are on the red side, so we use the red goal as our reference for both.
            closeShotDistance = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - CLOSE_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - CLOSE_SHOT_POSE.getY());
            farShotDistance = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - FAR_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - FAR_SHOT_POSE.getY());
        }

        // Using the classic linear interpolation formula y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double calculatedSpeed = CLOSE_SHOT_RPM + ((currentDistance - closeShotDistance) * (FAR_SHOT_RPM - CLOSE_SHOT_RPM)) / (farShotDistance - closeShotDistance);
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
