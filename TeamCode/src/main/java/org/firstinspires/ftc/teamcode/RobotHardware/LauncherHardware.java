package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherHardware {

    private DcMotorEx leftlauncher = null;
    private DcMotorEx rightlauncher = null;

    // These should be tuned for your robot
    static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.

    static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    public static final double RPM_TOLERANCE = 25; // Allowable error in RPM
    public static final double MAX_RPM = 6000; // Maximum RPM for the launcher
    public static final double TICKS_PER_REV = 28; // this is for the   GoBilda 6000rpm motor
    private double targetRPM = 0;

    public boolean launchClose = true;

    public void init(HardwareMap hardwareMap) {
        leftlauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightlauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        leftlauncher.setDirection(DcMotor.Direction.REVERSE);
        rightlauncher.setDirection(DcMotor.Direction.FORWARD);

        leftlauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightlauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Set custom PIDF coefficients if default tuning isn't good enough
        // PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        // leftlauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        // rightlauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        leftlauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    // --- Public Methods ---

    public void setTargetRPM(double rpm) {
        // This is a placeholder as TARGET_RPM is final. We need to change the implementation.
        // The spinUp method should accept the RPM directly.
    }

// CORRECTED approach:
// In LauncherHardware.java, change the spinUp() method to take a parameter.

    /** Spins up the flywheels to the default target RPM. Used by ActionManager. */
    public void spinUp() {

        if (launchClose){
            targetRPM = LAUNCHER_CLOSE_TARGET_VELOCITY;
        } else {
            targetRPM = LAUNCHER_FAR_TARGET_VELOCITY;
        }

        spinUp(targetRPM); // Calls the other spinUp method with the default value
    }

    /** Spins up the flywheels to a specific target RPM. Used by Classic TeleOp. */
    public void spinUp(double targetRPM) {
        double targetVelocity_TPS = (targetRPM * TICKS_PER_REV) / 60.0;
        rightlauncher.setVelocity(targetVelocity_TPS);
        leftlauncher.setVelocity(targetVelocity_TPS);
    }

    /** Switches between close and far launch velocities. */
    public void toggleLaunchDistance()
    {
        launchClose = !launchClose;
    }

    /** Reverses the launcher to clear jams. */
    public void reverse() {
        // Use a low, constant velocity for reverse
        rightlauncher.setVelocity(-1000);
        leftlauncher.setVelocity(-1000);
    }

    /** Stops the flywheels. */
    public void stop() {
        leftlauncher.setVelocity(0);
        rightlauncher.setVelocity(0);
    }

    /** Checks if both flywheels are at the target speed within tolerance. */
    public boolean isAtTargetSpeed(double targetRPM) {
        double leftError = Math.abs(targetRPM - getLeftFlywheelRPM());
        double rightError = Math.abs(targetRPM - getRightFlywheelRPM());
        return (leftError < RPM_TOLERANCE) && (rightError < RPM_TOLERANCE);
    }

    // And the default version for the ActionManager
    public boolean isAtTargetSpeed() {
        return isAtTargetSpeed(targetRPM);
    }

    public double getLeftFlywheelRPM() {
        return (leftlauncher.getVelocity() * 60.0 / TICKS_PER_REV);
    }

    public double getRightFlywheelRPM(){
        return (rightlauncher.getVelocity() * 60.0 / TICKS_PER_REV);
    }
}
