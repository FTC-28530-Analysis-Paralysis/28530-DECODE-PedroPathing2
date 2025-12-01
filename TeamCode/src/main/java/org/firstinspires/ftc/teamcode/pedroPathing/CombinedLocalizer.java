package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

/**
 * A fused localizer that orchestrates corrections between a dead-wheel localizer
 * (PinpointHardware) and a vision localizer (LimelightAprilTagLocalizer).
 */
public class CombinedLocalizer implements Localizer { 

    private final CustomPinpointLocalizer pinpoint;
    private final LimelightAprilTagLocalizer limelight;
    private final Telemetry telemetry;

    public CombinedLocalizer(CustomPinpointLocalizer pinpoint, LimelightAprilTagLocalizer limelight, Telemetry telemetry) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    @Override
    public void update() {
        pinpoint.update();
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelight.getRobotPoseWithLatency();
        if (limelightPoseData.isPresent()) {
            LimelightAprilTagLocalizer.LimelightPoseData data = limelightPoseData.get();
            Pose pinpointPoseAtLatency = pinpoint.getPoseAtLatency(data.latency);
            // CORRECTED: Use plus() and minus() as per the provided Pose.java file
            Pose error = data.pose.minus(pinpointPoseAtLatency);
            Pose correctedPose = pinpoint.getPose().plus(error);
            pinpoint.setPose(correctedPose);
            pinpoint.clearPoseHistory();
            telemetry.addData("Localization", "Applying vision correction.");
        }
    }

    // ======================================================================================
    // All other methods simply delegate to the underlying dead-wheel localizer.
    // ======================================================================================

    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPose(pose);
    }

    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
    }

    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return pinpoint.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return pinpoint.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return pinpoint.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        pinpoint.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return pinpoint.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }
}
