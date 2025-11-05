package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10); //TODO: Correct mass from 10 to whatever it is (kg)

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontDrive")
            .rightRearMotorName("RightBackDrive")
            .leftRearMotorName("LeftBackDrive")
            .leftFrontMotorName("LeftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5) //TODO: Correct forwardPodY from -5 to whatever it is (inches) to robot center of rotation. https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
            .strafePodX(0.5) //TODO: Correct strafePodX from 0.5 to whatever it is (inches) to robot center of rotation.
            .distanceUnit(DistanceUnit.INCH)
            //.forwardEncoderDirection(Encoder.REVERSE) //TODO: select & run localization test under the localization folder in the tuning OpMode, then move the robot forward. The x coordinate should increase.
            //.strafeEncoderDirection(Encoder.REVERSE) //TODO: Next move the robot left. The y coordinate should increase. If either of those does not happen, you must reverse the respective encoder (uncomment these lines)
            .hardwareMapName("pinpoint") //TODO: Correct hardware map name from "pinpoint" to whatever it is (String)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
