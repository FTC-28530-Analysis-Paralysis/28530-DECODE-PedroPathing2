package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="ButteTimeAuto", group="01 Butte", preselectTeleOp="Butte2Controller")
public class ButteTimeAuto extends OpMode{

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (runtime.seconds() < 5){
            //mecanumHardware.drive(.5, 0, 0);
        }
        telemetry.addData("Status", "Autonomous Running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Autonomous Finished");
        telemetry.update();
        //mecanumHardware.stop();


    }

}
