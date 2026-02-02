package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FeederHardware {
    public CRServo leftfeeder;
    public CRServo rightfeeder;
    private static final double FEEDER_POWER = 1.0;

    public void init(HardwareMap hardwareMap){
        leftfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightfeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftfeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        stop(); // Ensure it's off at init
    }

    // --- Public Methods ---

    /** Runs the left feeder feeder to move artifacts towards the launcher. */
    public void runLeft() {
        leftfeeder.setPower(FEEDER_POWER);
    }

    /** Runs the right feeder feeder to move artifacts towards the launcher. */
    public void runRight() {
        rightfeeder.setPower(FEEDER_POWER);
    }

    /** Reverses the feeder. */
    public void reverse() {
        leftfeeder.setPower(-FEEDER_POWER);
        rightfeeder.setPower(-FEEDER_POWER);

    }

    /** Stops the feeder. */
    public void stop() {
        leftfeeder.setPower(0.0);
        rightfeeder.setPower(0.0);
    }
}
