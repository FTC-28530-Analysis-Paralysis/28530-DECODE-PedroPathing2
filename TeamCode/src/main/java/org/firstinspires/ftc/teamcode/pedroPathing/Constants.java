package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Import our new custom constants class
import org.firstinspires.ftc.teamcode.pedroPathing.CustomPinpointConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)
            .forwardZeroPowerAcceleration(-28)
            .lateralZeroPowerAcceleration(-59)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.375, 0, .1, 0.06))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.73, 0.0254, 0.0001, 0.6, 0.00263))
            .centripetalScaling(0.0006);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.45, 1.28);

    public static class CustomDriveConstants {
        public final String LEFT_FRONT_MOTOR_NAME = "left_front_drive";
        public final String LEFT_BACK_MOTOR_NAME = "left_back_drive";
        public final String RIGHT_FRONT_MOTOR_NAME = "right_front_drive";
        public final String RIGHT_BACK_MOTOR_NAME = "right_back_drive";

        public final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction LEFT_BACK_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
        public final DcMotor.Direction RIGHT_BACK_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

        public double X_VELOCITY = 64.1809860589936;
        public double Y_VELOCITY = 52.84026090366634; // Set to X initially, tune if needed

        public final String VOLTAGE_SENSOR_NAME = "Control Hub";

        private final double[] convertToPolar = Pose.cartesianToPolar(X_VELOCITY, -Y_VELOCITY);
        public final Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

        public double maxPower = 1.0;
        public double motorCachingThreshold = 0.01;
        public boolean useBrakeModeInTeleOp = true;
        public boolean useVoltageCompensation = false;
        public double nominalVoltage = 12.0;
        public double staticFrictionCoefficient = 0.1;
    }

    public static CustomDriveConstants customDriveConstants = new CustomDriveConstants();

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.25)
            .strafePodX(-6.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /**
     * Creates a new Follower instance with a complete, fused localization system.
     *
     * @param hardwareMap The hardwareMap from the OpMode.
     * @param localizer The localizer instance for the follower to use.
     * @return A fully initialized Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.setLocalizer(localizer) // Use the provided localizer
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, customDriveConstants))
                .build();
    }
}
