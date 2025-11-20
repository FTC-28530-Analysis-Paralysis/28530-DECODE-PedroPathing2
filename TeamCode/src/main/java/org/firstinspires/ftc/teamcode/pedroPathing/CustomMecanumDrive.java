package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class CustomMecanumDrive extends Drivetrain {

    private final Constants.CustomDriveConstants constants;
    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    public CustomMecanumDrive(HardwareMap hardwareMap, Constants.CustomDriveConstants constants) {
        this.constants = constants;

        this.maxPowerScaling = constants.maxPower;
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.get(constants.VOLTAGE_SENSOR_NAME);

        leftFront = hardwareMap.get(DcMotorEx.class, constants.LEFT_FRONT_MOTOR_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, constants.LEFT_BACK_MOTOR_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, constants.RIGHT_BACK_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, constants.RIGHT_FRONT_MOTOR_NAME);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        updateConstants(); // Set motor directions
        setMotorsToFloat();
        breakFollowing();
    }

    @Override
    public void updateConstants() {
        leftFront.setDirection(constants.LEFT_FRONT_MOTOR_DIRECTION);
        leftRear.setDirection(constants.LEFT_BACK_MOTOR_DIRECTION);
        rightFront.setDirection(constants.RIGHT_FRONT_MOTOR_DIRECTION);
        rightRear.setDirection(constants.RIGHT_BACK_MOTOR_DIRECTION);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Combine the input vectors to get a single target translational vector.
        Vector totalTranslational = pathingPower.plus(correctivePower);

        // THE FIX for Rotated Driving:
        // As you diagnosed, the library's vectors are rotated 90 degrees. "Up is right, right is back."
        // This corrects for that by swapping the components and negating one.
        double robotForward = -totalTranslational.getXComponent();
        double robotStrafe = totalTranslational.getYComponent();

        // The turn power is taken from the heading vector, and negated to fix the reversal you saw.
        double turn = -headingPower.getXComponent();

        // THE FIX for Strafing:
        // These are the exact algebraic formulas from your own working `KalispellTeleop`.
        // This abandons the complex library math and uses the logic you have already proven works on your robot.
        // This is the simplest way to "switch the strafing values" as you requested.
        double[] motorPowers = new double[4];
        motorPowers[0] = robotForward + robotStrafe + turn; // Left Front
        motorPowers[1] = robotForward - robotStrafe + turn; // Left Rear
        motorPowers[2] = robotForward + robotStrafe - turn; // Right Front
        motorPowers[3] = robotForward - robotStrafe - turn; // Right Rear

        return motorPowers;
    }

    @Override
    public void runDrive(double[] drivePowers) {
        // Normalize the motor powers to ensure they are within the -1.0 to 1.0 range.
        double max = 1.0;
        for (double power : drivePowers) {
            if (Math.abs(power) > max) {
                max = Math.abs(power);
            }
        }
        if (max > 1) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] /= max;
            }
        }

        for (int i = 0; i < motors.size(); i++) {
            // Only send a new power command if it's different enough from the previous one.
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) setMotorsToBrake();
        else setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) setMotorsToBrake();
        else setMotorsToFloat();
    }

    @Override
    public double xVelocity() {
        return constants.X_VELOCITY;
    }

    @Override
    public double yVelocity() {
        return constants.Y_VELOCITY;
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.X_VELOCITY = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        constants.Y_VELOCITY = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return String.format("LF: %.2f, LR: %.2f, RF: %.2f, RR: %.2f",
                leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower());
    }

    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }
}
