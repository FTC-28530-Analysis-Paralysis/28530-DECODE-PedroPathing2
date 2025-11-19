package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferHardware {
    public CRServo transferServo;
    public static enum Direction {FORWARD, REVERSE};

    public void init(HardwareMap hardwareMap){
        transferServo = hardwareMap.get(CRServo.class, "transfer");
        transferServo.setPower(0.0);
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Transfer(Direction direction, boolean transferOn){
        if(transferOn && direction == Direction.FORWARD){
            transferServo.setPower(1.0);
        }else if(transferOn && direction == Direction.REVERSE) {
            transferServo.setPower(-1.0);
        } else {
            transferServo.setPower(0.0);
        }
    }
}

