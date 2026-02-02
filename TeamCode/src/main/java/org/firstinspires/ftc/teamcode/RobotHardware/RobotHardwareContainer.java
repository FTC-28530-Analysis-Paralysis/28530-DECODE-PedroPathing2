package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class acts as a container to hold and initialize all the robot's mechanical hardware subsystems.
 * The localization system is now handled separately by the Follower in the Constants class.
 */
public class RobotHardwareContainer {

    // Publicly accessible hardware subsystem objects
    public final IntakeHardware intake;
    public final LauncherHardware launcher;
    public final FeederHardware feeder;
    public final IndicatorLightHardware indicatorLight;

    // The ColorDiverter is nullable because it may not exist on all robot configurations.
    public ColorDiverterHardware colorDiverter;

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create instances of each hardware class
        intake = new IntakeHardware();
        launcher = new LauncherHardware();
        feeder = new FeederHardware();
        indicatorLight = new IndicatorLightHardware();
        colorDiverter = new ColorDiverterHardware();

        // Call the init() method for each mechanical subsystem
        indicatorLight.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap, indicatorLight);
        feeder.init(hardwareMap);
        colorDiverter.init(hardwareMap);
    }
}
