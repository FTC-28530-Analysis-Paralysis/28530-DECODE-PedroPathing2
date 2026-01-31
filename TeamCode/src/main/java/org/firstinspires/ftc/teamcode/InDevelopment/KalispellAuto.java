//package org.firstinspires.ftc.teamcode.InDevelopment;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
////import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
////import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
////import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;
//
//
//@Autonomous(name="KalispellAuto", group="01 Kalispell")
//public class KalispellAuto extends OpMode{
//    straferDrive = new StraferDrive();
////    IntakeHardware intakeHardware = new IntakeHardware();
////    MecanumHardware mecanumHardware = new MecanumHardware();
////    TransferHardware transferHardware = new TransferHardware();
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void init(){
//        straferDrive.init(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    @Override
//    public void loop() {
//        if (runtime.seconds() < 2) {
//            straferDrive.drive(.7, 0, 0);
//            telemetry.addData("Status", "Autonomous Running");
//        } else {
//            straferDrive.stop();
//            telemetry.addData("Status", "Autonomous Finished");
//        }
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        straferDrive.stop();
//    }
//
//}
