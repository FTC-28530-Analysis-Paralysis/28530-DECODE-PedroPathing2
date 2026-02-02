package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

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
        setMotorsToBrake();
        breakFollowing();

        // This section defines the physical reality of the drive wheels for the library's math.
        // The `vectors` array holds the direction of force each wheel exerts when spun forward.
        Vector standardVector = constants.frontLeftVector.normalize();
        Vector oppositeVector = new Vector(standardVector.getMagnitude(), 2 * Math.PI - standardVector.getTheta());

        // THE FIX for incorrect strafing:
        // Your robot's behavior indicates a non-standard drive configuration where the
        // right-side wheels need their strafe behavior inverted. In the library's vector math,
        // the way to "flip the negative for strafe" is to swap the force vectors for the right wheels.
        // This change tells the complex math the truth about how your specific robot hardware behaves.
        vectors = new Vector[]{
                standardVector, // Left Front (Standard)
                oppositeVector, // Left Rear (Standard)
                standardVector, // Right Front (CHANGED from oppositeVector to fix your robot's strafing)
                oppositeVector  // Right Rear (CHANGED from standardVector to fix your robot's strafing)
        };
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
        // This method is a direct, faithful adaptation of the official PedroPathing Mecanum.java source code.

        // --- Step 1: Clamp all incoming power vectors ---
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        double[] wheelPowers = new double[4];
        Vector[] mecanumVectorsCopy = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];

        // --- Step 2: Combine all power vectors ---
        if (correctivePower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        // --- Step 3: Scale up the final left and right target vectors ---
        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        // --- Step 4: Rotate the physical wheel force vectors to match the robot's current heading. ---
        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = vectors[i].copy();
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        // --- Step 5: The core of the math. ---
        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        // --- Step 6: Apply voltage compensation if enabled. ---
        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        // --- Step 7: Normalize all wheel powers to ensure none exceed the maximum allowed power. ---
        double wheelPowerMax = 0;
        for (double power : wheelPowers) {
            if(Math.abs(power) > wheelPowerMax) wheelPowerMax = Math.abs(power);
        }

        if (wheelPowerMax > maxPowerScaling) {
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] = (wheelPowers[i] / wheelPowerMax) * maxPowerScaling;
            }
        }

        return wheelPowers;
    }

    @Override
    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
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
