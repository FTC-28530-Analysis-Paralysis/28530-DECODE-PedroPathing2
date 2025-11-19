package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeHardware {

    private DcMotorEx intakeMotor = null;
    public static enum Direction {FORWARD, REVERSE};

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "activeIntake");

        //find out what you really have to do for this
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Intake(Direction direction, boolean intakeOn){
        if(intakeOn && direction == Direction.FORWARD){

            intakeMotor.setPower(1.0);
        }else if(intakeOn && direction == Direction.REVERSE) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
    }
}





