package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Autonomous(name = "ConfigurableAuto Limelight", group = "01 Helena", preselectTeleOp = "TeleopManualControls")
public class ConfigurableAuto_Limelight extends OpMode {

    // ========== CONFIGURATION ==========
    private enum Alliance { BLUE, RED }
    private enum StartPosition { FRONT, BACK }
    private enum Spike { FRONT, MIDDLE, BACK } // Renamed for clarity

    private Alliance alliance = Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    // For this advanced auto, we will always intake from a specific spike mark.
    private Spike intakeSpike = Spike.FRONT;

    private boolean dpad_up_down_pressed = false;
    private boolean bumper_pressed = false;
    private boolean a_b_x_pressed = false;

    // ========== OPMODE MEMBERS ==========
    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private RobotHardwareContainer robot;
    private ActionManager actionManager;

    // === PATHING ===
    private Pose startPose, scorePose, parkPose, initialSpikePose;
    private List<Character> colorOrder = new ArrayList<>();
    private int currentBallOf3 = 0;

    private int pathState;

    // Defines the order of pixels on each spike mark from the audience perspective.
    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
    private final double INTAKE_OFFSET_DISTANCE = 6.0; // Inches to move between pixels

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap, robot.localizer);

        telemetry.addLine("--- Advanced Auto Configuration ---");
        telemetry.addLine("D-Pad: Alliance | Bumpers: Start Pos");
        telemetry.addLine("A/B/X: Intake Spike (Front/Mid/Back)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !dpad_up_down_pressed) alliance = Alliance.BLUE;
        if (gamepad1.dpad_down && !dpad_up_down_pressed) alliance = Alliance.RED;
        dpad_up_down_pressed = gamepad1.dpad_up || gamepad1.dpad_down;

        if (gamepad1.left_bumper && !bumper_pressed) startPosition = StartPosition.FRONT;
        if (gamepad1.right_bumper && !bumper_pressed) startPosition = StartPosition.BACK;
        bumper_pressed = gamepad1.left_bumper || gamepad1.right_bumper;

        if (gamepad1.a && !a_b_x_pressed) intakeSpike = Spike.FRONT;
        if (gamepad1.b && !a_b_x_pressed) intakeSpike = Spike.MIDDLE;
        if (gamepad1.x && !a_b_x_pressed) intakeSpike = Spike.BACK;
        a_b_x_pressed = gamepad1.a || gamepad1.b || gamepad1.x;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start", startPosition);
        telemetry.addData("Intake Spike", intakeSpike);
        telemetry.update();
    }

    @Override
    public void start() {
        calculatePoses();
        follower.setStartingPose(startPose);
        setPathState(1);
    }

    @Override
    public void loop() {
        follower.update();
        actionManager.update();
        updatePath();
        telemetry.addData("Path State", pathState).addData("Ball", currentBallOf3);
        telemetry.addData("X", follower.getPose().getX()).addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    private void calculatePoses() {
        if (alliance == Alliance.BLUE) {
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
            scorePose = FieldPosePresets.BLUE_SCORE_POSE;
            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
            switch (intakeSpike) {
                case FRONT: initialSpikePose = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE; colorOrder = SPIKE_FRONT_COLORS; break;
                case MIDDLE: initialSpikePose = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE; colorOrder = SPIKE_MIDDLE_COLORS; break;
                case BACK: initialSpikePose = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE; colorOrder = SPIKE_BACK_COLORS; break;
            }
        } else { // RED Alliance
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
            scorePose = FieldPosePresets.RED_SCORE_POSE;
            parkPose = FieldPosePresets.RED_AUTO_PARK;
            switch (intakeSpike) {
                case FRONT: initialSpikePose = FieldPosePresets.RED_PICKUP_FRONT_SPIKE; colorOrder = SPIKE_FRONT_COLORS; break;
                case MIDDLE: initialSpikePose = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE; colorOrder = SPIKE_MIDDLE_COLORS; break;
                case BACK: initialSpikePose = FieldPosePresets.RED_PICKUP_BACK_SPIKE; colorOrder = SPIKE_BACK_COLORS; break;
            }
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        timer.reset();
    }

    private void updatePath() {
        switch (pathState) {
            case 0: break; // IDLE

            case 1: // Drive to score pre-loaded pixel
                follower.followPath(new Path(new BezierLine(startPose, scorePose)));
                setPathState(2);
                break;

            case 2: // Wait for path, then launch
                if (!follower.isBusy()) {
                    actionManager.startLaunch();
                    setPathState(3);
                }
                break;

            case 3: // Wait for launch, then start intake sequence
                if (!actionManager.isBusy()) {
                    currentBallOf3 = 0;
                    follower.followPath(new Path(new BezierLine(scorePose, initialSpikePose)));
                    setPathState(4); // Begin the intake dance
                }
                break;

            // --- Intake Sub-Sequence ---
            case 4: // Arrived at a spike mark line, start the dance
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // Set diverter for the current ball
                char expectedColor = colorOrder.get(currentBallOf3);
                if (expectedColor == 'P') actionManager.setDiverterLeft();
                else actionManager.setDiverterRight();
                setPathState(6);
                break;

            case 6: // Start intake and wait briefly
                actionManager.startIntake(); // This is a timed action now
                setPathState(7);
                break;
            
            case 7: // Wait for intake action, then check if we are done
                if (!actionManager.isBusy()) {
                    currentBallOf3++; // Move to the next ball
                    if (currentBallOf3 >= 3) { // Done with all 3 balls
                        // Start driving to the scoring pose while the last intake finishes
                        follower.followPath(new Path(new BezierLine(follower.getPose(), scorePose)));
                        setPathState(10); // Go to final scoring state
                    } else {
                        // Not done, move sideways to the next ball
                        double offset = (alliance == Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
                        Pose nextBallPose = follower.getPose().plus(new Pose(0, offset, 0));
                        follower.followPath(new Path(new BezierLine(follower.getPose(), nextBallPose)));
                        setPathState(4); // Go back to start of dance for next ball
                    }
                }
                break;
            
            // --- End of Main Sequence ---
            case 10: // Arrived at score pose after intake sequence
                if (!follower.isBusy()) {
                    actionManager.startLaunch();
                    setPathState(11);
                }
                break;

            case 11: // Wait for final launch
                if (!actionManager.isBusy()) {
                    follower.followPath(new Path(new BezierLine(follower.getPose(), parkPose)));
                    setPathState(12);
                }
                break;

            case 12: // Wait for park to finish
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                follower.breakFollowing();
                break;
        }
    }
}
