package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.FeederHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.IndicatorLightHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;

@TeleOp(name = "Launcher Speed Tuning", group = "02 Tuning")
public class LauncherTuningOpMode extends OpMode {

    private LauncherHardware launcher;
    private IntakeHardware intake;
    private FeederHardware feeder;
    private IndicatorLightHardware light;

    // --- Tuning Variables ---
    private double targetRPM = 3000.0; // Starting RPM
    private final double RPM_INCREMENT = 20.0; // Adjust RPM by this much
    private boolean launcherOn = false;

    @Override
    public void init() {
        // We don't need a follower for this tuning OpMode
        launcher = new LauncherHardware();
        intake = new IntakeHardware();
        feeder = new FeederHardware();
        light = new IndicatorLightHardware();

        launcher.init(hardwareMap, light);
        intake.init(hardwareMap);
        feeder.init(hardwareMap);

        telemetry.addLine("Launcher Tuning OpMode Initialized.");
        telemetry.addLine("A: Toggle Launcher | D-Pad U/D: Adjust RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleLauncherControls();
        handleMechanismControls();
        updateTelemetry();
    }

    private void handleLauncherControls() {
        if (gamepad1.aWasPressed()) {
            launcherOn = !launcherOn;
            if (launcherOn) {
                launcher.setLaunchSpeed(targetRPM);
            } else {
                launcher.stop();
            }
        }

        if (launcherOn) {
            if (gamepad1.dpadUpWasPressed()) {
                targetRPM += RPM_INCREMENT;
                launcher.setLaunchSpeed(targetRPM);
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetRPM -= RPM_INCREMENT;
                launcher.setLaunchSpeed(targetRPM);
            }
        }
    }

    private void handleMechanismControls() {
        // --- Intake ---
        if (gamepad1.left_trigger > .1) {
            intake.run();
        } else {
            intake.stop();
        }

        // --- Transfer/Feeder--
        if (gamepad1.left_bumper) {
            feeder.runLeft();
        } else {
            feeder.stop();
        }
        if (gamepad1.right_bumper) {
            feeder.runRight();
        } else {
            feeder.stop();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Launcher State", launcherOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine("------------");
        telemetry.addData("Left Flywheel RPM", "%.0f", launcher.getLeftFlywheelRPM());
        telemetry.addData("Right Flywheel RPM", "%.0f", launcher.getRightFlywheelRPM());
        telemetry.addLine("------------");
        telemetry.addData("Left Flywheel Current (A)", "%.2f", launcher.getLeftFlywheelCurrent());
        telemetry.addData("Right Flywheel Current (A)", "%.2f", launcher.getRightFlywheelCurrent());
        telemetry.update();
    }
}
