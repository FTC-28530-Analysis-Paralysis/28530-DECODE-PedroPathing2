package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeHardware {

    private DcMotorEx intake = null;
    private static final double INTAKE_POWER = 1.0;
    private boolean intakeOn = false;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop(); // Ensure motor is off at init
    }

    // --- Public Methods ---

    /** Runs the intake inwards to collect pixels. */
    public void run() {
        intake.setPower(INTAKE_POWER);
    }

    /** Reverses the intake to eject pixels. */
    public void reverse() {
        intake.setPower(-INTAKE_POWER);
    }

    /** Stops the intake motor. */
    public void stop() {
        intake.setPower(0.0);
    }

    public void toggleIntake(){
        intakeOn = !intakeOn;
        if (intakeOn) run();
        else stop();
    }
}
