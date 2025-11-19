package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;

@TeleOp(name="KalispellTeleop", group="01 Kalispell")
public class KalispellTeleop extends OpMode {
    LauncherHardware launcherHardware = new LauncherHardware();
    IntakeHardware intakeHardware = new IntakeHardware();
    MecanumHardware mecanumHardware = new MecanumHardware();
    TransferHardware transferHardware = new TransferHardware();
    public boolean shootersOn = false;
    private boolean aButtonPressed = false;
    public double targetShooterSpeed_RPM = 2000;
    public double closetargetShooterSpeed_RPM = 2000;
    public double farTargetShooterSpeed_RPM = 2000;

    private ElapsedTime runtime = new ElapsedTime();
    private double leftBumperLastTime = 0;
    private double rightBumperLastTime = 0;
    private static int SHOOTER_SPEED_INCREMENT = 100;

    @Override
    public void init(){
        launcherHardware.init(hardwareMap);
        intakeHardware.init(hardwareMap);
        mecanumHardware.init(hardwareMap);
        transferHardware.init(hardwareMap);
    }

    @Override
    public void loop(){
        ///  potentially adding another controller
        ///  one for driving and intake, another for shooting and transfer system possibly
            // Driving
            double forward = -gamepad1.left_stick_y;  // Controls forward and backward movement
            double strafe = gamepad1.left_stick_x;   // Controls side-to-side movement
            double turn = gamepad1.right_stick_x;    // Controls turning/rotation
            mecanumHardware.drive(forward, strafe, turn);
            // Intake
            if (gamepad1.dpad_right){
                intakeHardware.Intake(IntakeHardware.Direction.FORWARD, true);
            }else if (gamepad1.dpad_left){
                intakeHardware.Intake(IntakeHardware.Direction.REVERSE, true);
            }else if (gamepad1.dpad_down){
                intakeHardware.Intake(IntakeHardware.Direction.FORWARD, false);

            }
            // Launcher flywheel speed
            if (gamepad1.right_bumper) {
                if (runtime.seconds() - rightBumperLastTime > .075) {
                    if (targetShooterSpeed_RPM < LauncherHardware.MAX_SHOOTER_SPEED) {
                        targetShooterSpeed_RPM += SHOOTER_SPEED_INCREMENT;
                    }
                    rightBumperLastTime = runtime.seconds();
                }
            } else if (gamepad1.left_bumper) {
                if (runtime.seconds() - leftBumperLastTime > .075) {
                    if (targetShooterSpeed_RPM > 0) {
                        targetShooterSpeed_RPM -= SHOOTER_SPEED_INCREMENT;
                    }
                    leftBumperLastTime = runtime.seconds();
                }
            }

            // reverses launchers hopefully

            if(gamepad1.b) {
                launcherHardware.reverseRunLauncher(true);
            }else{
                launcherHardware.reverseRunLauncher(false);
            }

            // Toggle shooter
            if (gamepad1.a) {
                if (!aButtonPressed) {
                    aButtonPressed = true;
                    shootersOn = !shootersOn;
                }
            } else {
                aButtonPressed = false;
            }


            if (shootersOn) {
                launcherHardware.runLauncher(targetShooterSpeed_RPM);
            } else {
                launcherHardware.runLauncher(0);
            }

            // Ball transfer
            if(gamepad1.right_trigger > 0.1){
                transferHardware.Transfer(TransferHardware.Direction.FORWARD, true);
            }else if (gamepad1.left_trigger > 0.1){
                transferHardware.Transfer(TransferHardware.Direction.REVERSE, true);
            } else{
                transferHardware.Transfer(TransferHardware.Direction.FORWARD, false);
            }
            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Launcher Target Speed", targetShooterSpeed_RPM);
            telemetry.addData("LeftFlywheel RPM",  "%.0f", launcherHardware.getLeftFlywheelRPM());
            telemetry.addData("Close Triangle Target RPM",  "%.0f", closetargetShooterSpeed_RPM);
            telemetry.addData("Far Triangle Target RPM",  "%.0f", farTargetShooterSpeed_RPM);

        telemetry.update();
    }

    public void stop(){
        transferHardware.Transfer(TransferHardware.Direction.FORWARD, false);
        intakeHardware.Intake(IntakeHardware.Direction.FORWARD, false);
        launcherHardware.stop();
    }

}



