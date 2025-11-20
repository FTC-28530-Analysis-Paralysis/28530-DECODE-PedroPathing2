package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// NEW: Import the hardware container and the action manager
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware; // Mecanum drive is separate

@TeleOp(name="ButteTeleop", group="01 Butte")
public class TeleopWithActions extends OpMode {

    // --- Declare OpMode members ---

    // The container for all our mechanisms (intake, launcher, transfer)
    RobotHardwareContainer robot;
    // The manager for all our mechanism actions
    ActionManager actionManager;
    // The drive train hardware (can be kept separate for clarity)
    MecanumHardware mecanumHardware;

    // --- Private state variables for button presses ---
    private boolean dpad_right_pressed = false;
    private boolean y_pressed = false;

    @Override
    public void init(){
        // Initialize the hardware container - this inits Intake, Launcher, and Transfer all at once!
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        // Initialize the ActionManager with the hardware container
        actionManager = new ActionManager(robot);
        // Initialize the drivetrain
        mecanumHardware = new MecanumHardware();
        mecanumHardware.init(hardwareMap);

        telemetry.addLine("Robot is initialized and ready.");
        telemetry.update();
    }

    @Override
    public void loop(){
        // --- UPDATE MANAGERS ---
        // This is CRITICAL. The action manager's state machine must be updated every loop.
        actionManager.update();

        // --- DRIVER CONTROLS (Gamepad 1) ---

        // Drivetrain
        double forward = -gamepad1.left_stick_y;  // Forward/Backward
        double strafe = gamepad1.left_stick_x;   // Left/Right
        double turn = gamepad1.right_stick_x;    // Rotation
        mecanumHardware.drive(forward, strafe, turn);

        // Intake Controls
        if (gamepad1.right_trigger > 0.1) {
            // Right trigger runs the intake IN
            actionManager.startIntake();
        } else if (gamepad1.left_trigger > 0.1) {
            // Left trigger runs the intake and transfer in REVERSE to clear jams
            // (We will add a reverse method to ActionManager for this)
            robot.intake.reverse();
            robot.transfer.reverse();
        } else {
            // If no intake buttons are pressed, and the manager isn't busy with another task, stop the intake.
            if (!actionManager.isBusy()) {
                robot.intake.stop();
                robot.transfer.stop();
            }
        }

        // Scoring Controls
        // D-pad Right starts the FULL scoring sequence (spin-up, then launch)
        if (gamepad1.dpad_right && !dpad_right_pressed) {
            actionManager.startScoring();
        }

        // Transfer Control
        // 'Y' button starts the transfer action (runs intake and transfer for a set time)
        if (gamepad1.y && !y_pressed) {
            actionManager.startTransfer();
        }

        // Manual Override / Stop All
        // 'B' button is our emergency stop. It will halt all actions.
        if (gamepad1.b) {
            actionManager.stopAll();
        }

        // --- Update button state trackers ---
        dpad_right_pressed = gamepad1.dpad_right;
        y_pressed = gamepad1.y;


        // --- TELEMETRY ---
        telemetry.addData("Action Manager State", actionManager.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Left Flywheel RPM", "%.0f", robot.launcher.getLeftFlywheelRPM());
        telemetry.addData("Right Flywheel RPM", "%.0f", robot.launcher.getRightFlywheelRPM());
        telemetry.addData("Target RPM", robot.launcher.TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void stop(){
        // Ensure everything is stopped when the OpMode ends.
        actionManager.stopAll();
        mecanumHardware.drive(0,0,0);
    }
}
