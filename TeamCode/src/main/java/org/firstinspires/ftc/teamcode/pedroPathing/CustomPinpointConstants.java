package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * This is our own custom constants class for our PinpointLocalizer.
 * This follows the design pattern recommended by the PedroPathing documentation,
 * decoupling our implementation from the library's internal constants.
 */
public class CustomPinpointConstants {

    public double forwardPodY = -3.25;
    public double strafePodX = -6.625;
    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public String hardwareMapName = "pinpoint";
    public OptionalDouble yawScalar = OptionalDouble.empty();
    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public CustomPinpointConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public CustomPinpointConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public CustomPinpointConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public CustomPinpointConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public CustomPinpointConstants yawScalar(double yawScalar) {
        this.yawScalar = OptionalDouble.of(yawScalar);
        return this;
    }

    public CustomPinpointConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public CustomPinpointConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public CustomPinpointConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }
}
