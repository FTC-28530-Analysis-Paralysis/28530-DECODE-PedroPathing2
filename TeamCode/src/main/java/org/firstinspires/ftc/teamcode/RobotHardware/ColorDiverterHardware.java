package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware class for the color-sorting diverter gate servo.
 */
public class ColorDiverterHardware {

    private Servo diverterServo;

    // Define the servo positions for each track. These are placeholders and must be tuned.
    public static final double LEFT_TRACK_POSITION = 0.2;
    public static final double RIGHT_TRACK_POSITION = 0.8;

    public enum GatePosition {
        LEFT,  // Directs pixels to the left track
        RIGHT  // Directs pixels to the right track
    }

    public void init(HardwareMap hardwareMap) {
        diverterServo = hardwareMap.get(Servo.class, "colorDiverter");
    }

    /**
     * Sets the diverter gate to the specified position.
     * @param position The target position (LEFT or RIGHT).
     */
    public void setPosition(GatePosition position) {
        if (position == GatePosition.LEFT) {
            diverterServo.setPosition(LEFT_TRACK_POSITION);
        } else {
            diverterServo.setPosition(RIGHT_TRACK_POSITION);
        }
    }
}
