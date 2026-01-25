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

    private boolean isPoseReliable = false;

    // Running totals for error correction diagnostics
    private double totalTranslationalError = 0.0;
    private double totalHeadingError = 0.0;

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

            // Validity check to prevent corrupting the pose with an invalid (0,0,0) from Limelight
            if (data.pose.getX() == 0 && data.pose.getY() == 0 && data.pose.getHeading() == 0) {
                telemetry.addData("Localization", "Rejected invalid (0,0,0) vision pose.");
                return; // Do not apply this invalid correction
            }

            Pose pinpointPoseAtLatency = pinpoint.getPoseAtLatency(data.latency);
            Pose error = data.pose.minus(pinpointPoseAtLatency);

            // Calculate and accumulate the error for diagnostics
            totalTranslationalError += Math.hypot(error.getX(), error.getY());
            totalHeadingError += Math.abs(Math.toDegrees(error.getHeading()));

            Pose correctedPose = pinpoint.getPose().plus(error);
            setPose(correctedPose); // Use this.setPose() to ensure reliability flag is set
            pinpoint.clearPoseHistory();
            telemetry.addData("Localization", "Applying vision correction.");
        } else {
            telemetry.addData("Localization", "No vision data.");
        }

        // Always display the running totals for diagnostics
        telemetry.addData("Total Translation Error", "%.2f in", totalTranslationalError);
        telemetry.addData("Total Heading Error", "%.2f deg", totalHeadingError);
    }

    /**
     * Returns true if the current pose is considered reliable (i.e., not a placeholder).
     */
    public boolean isPoseReliable() {
        return isPoseReliable;
    }

    /**
     * Resets the robot's heading. If the pose is unknown, it creates an unreliable placeholder.
     */
    public void resetHeading() {
        Pose currentPose = getPose();
        if (currentPose == null) {
            // We have no pose, so we create a placeholder. This is NOT reliable.
            setPose(new Pose(0, 0, Math.toRadians(90)));
            isPoseReliable = false; // Explicitly mark as unreliable
        } else {
            // We have a real pose, just reset the heading. The position is still good.
            setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));
            // isPoseReliable will be set to true by the call to setPose()
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
        isPoseReliable = true; // Any time we explicitly set a pose, we assume it's reliable.
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
        isPoseReliable = true;
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
