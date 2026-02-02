//package org.firstinspires.ftc.teamcode.InDevelopment;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.geometry.BezierLine;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
//import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
//import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@Autonomous(name = "ConfigurableAuto Limelight", group = "01 Helena", preselectTeleOp = "TeleopManualControls")
//public class ConfigurableAuto_Limelight extends OpMode {
//
//    // ========== CONFIGURATION ==========
//    private enum Alliance { BLUE, RED }
//    private enum StartPosition { FRONT, BACK }
//
//    // Command-based structure for building a sequence.
//    private enum AutoCommand {
//        GO_TO_FRONT_SPIKE,
//        GO_TO_MIDDLE_SPIKE,
//        GO_TO_BACK_SPIKE,
//        INTAKE_CYCLE, // Perform the 3-pixel intake dance
//        SCORE,
//        HIT_GATE,
//        PARK
//    }
//
//    private enum SpikeLocation { NONE, FRONT, MIDDLE, BACK } // To track context for INTAKE
//
//    private Alliance alliance = Alliance.BLUE;
//    private StartPosition startPosition = StartPosition.FRONT;
//    private SpikeLocation currentSpikeContext = SpikeLocation.NONE;
//
//    private ArrayList<AutoCommand> autoCommands = new ArrayList<>();
//    private int commandMenuIndex = 0;
//    private int currentCommandIndex = 0;
//
//    private boolean dpad_up_down_pressed = false;
//    private boolean dpad_left_right_pressed = false;
//    private boolean bumper_pressed = false;
//    private boolean a_pressed = false;
//    private boolean y_pressed = false;
//
//    // ========== OPMODE MEMBERS ==========
//    private Follower follower;
//    private ElapsedTime timer = new ElapsedTime();
//    private RobotHardwareContainer robot;
//    private ActionManager actionManager;
//
//    // === PATHING & STATE ===
//    private Pose startPose, scorePose, parkPose;
//    private Pose frontSpike, middleSpike, backSpike;
//    private Pose gateApproachPose, gateTriggerPose;
//    private int pathState = 0;
//
//    // Intake cycle variables
//    private int intakeCycleBallCount = 0;
//    private List<Character> intakeColorOrder = new ArrayList<>();
//    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
//    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
//    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
//    private final double INTAKE_OFFSET_DISTANCE = 6.0;
//
//
//    @Override
//    public void init() {
//        robot = new RobotHardwareContainer(hardwareMap, telemetry);
//        actionManager = new ActionManager(robot);
//        follower = Constants.createFollower(hardwareMap, robot.localizer);
//
//        telemetry.addLine("--- Playlist Autonomous Builder ---");
//        telemetry.addLine("D-Pad U/D: Alliance | Bumpers: Start Pos");
//        telemetry.addLine("D-Pad L/R: Select Command");
//        telemetry.addLine("A: Add Command | Y: Clear Playlist");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        // --- Basic Config ---
//        if (gamepad1.dpad_up && !dpad_up_down_pressed) alliance = Alliance.BLUE;
//        if (gamepad1.dpad_down && !dpad_up_down_pressed) alliance = Alliance.RED;
//        dpad_up_down_pressed = gamepad1.dpad_up || gamepad1.dpad_down;
//
//        if (gamepad1.left_bumper && !bumper_pressed) startPosition = StartPosition.FRONT;
//        if (gamepad1.right_bumper && !bumper_pressed) startPosition = StartPosition.BACK;
//        bumper_pressed = gamepad1.left_bumper || gamepad1.right_bumper;
//
//        // --- Playlist Builder ---
//        AutoCommand[] allCommands = AutoCommand.values();
//        if (gamepad1.dpad_right && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex + 1) % allCommands.length;
//        if (gamepad1.dpad_left && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex - 1 + allCommands.length) % allCommands.length;
//        dpad_left_right_pressed = gamepad1.dpad_left || gamepad1.dpad_right;
//
//        if (gamepad1.a && !a_pressed) autoCommands.add(allCommands[commandMenuIndex]);
//        a_pressed = gamepad1.a;
//
//        if (gamepad1.y && !y_pressed) autoCommands.clear();
//        y_pressed = gamepad1.y;
//
//        // --- Telemetry ---
//        telemetry.addData("Alliance", alliance).addData("Start", startPosition);
//        telemetry.addLine("\n--- Build Your Playlist ---");
//        telemetry.addData("--> Selected Command", allCommands[commandMenuIndex]);
//        telemetry.addLine("\nCurrent Playlist:");
//        for (int i = 0; i < autoCommands.size(); i++) {
//            telemetry.addLine((i+1) + ". " + autoCommands.get(i));
//        }
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        calculatePoses();
//        follower.setStartingPose(startPose);
//        currentCommandIndex = 0;
//        if (!autoCommands.isEmpty()) {
//            setPathState(1);
//        } else {
//            setPathState(-1);
//        }
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        actionManager.update();
//        updatePath();
//        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
//        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
//        telemetry.addData("Path State", pathState);
//        telemetry.update();
//    }
//
//    private void calculatePoses() {
//        if (alliance == Alliance.BLUE) {
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
//            scorePose = FieldPosePresets.BLUE_SCORE_CLOSE_TO_GOAL;
//            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
//            frontSpike = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
//            middleSpike = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
//            backSpike = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
//            gateApproachPose = FieldPosePresets.BLUE_GATE_APPROACH;
//            gateTriggerPose = FieldPosePresets.BLUE_GATE_TRIGGER;
//        } else { // RED Alliance
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
//            scorePose = FieldPosePresets.RED_SCORE_CLOSE_TO_GOAL;
//            parkPose = FieldPosePresets.RED_AUTO_PARK;
//            frontSpike = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
//            middleSpike = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
//            backSpike = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
//            gateApproachPose = FieldPosePresets.RED_GATE_APPROACH;
//            gateTriggerPose = FieldPosePresets.RED_GATE_TRIGGER;
//        }
//    }
//
//    private void setPathState(int newState) {
//        pathState = newState;
//        timer.reset();
//    }
//
//    private void updatePath() {
//        if (currentCommandIndex >= autoCommands.size()) {
//            setPathState(-1); // Done with playlist
//            return;
//        }
//
//        switch (pathState) {
//            case 0: break; // IDLE
//
//            case 1: executeCommand(autoCommands.get(currentCommandIndex)); break;
//
//            case 2: // Wait for simple path/action to complete
//                if (!follower.isBusy() && !actionManager.isBusy()) advanceToNextCommand();
//                break;
//
//            // --- Scoring Sub-States ---
//            case 100: if (!follower.isBusy()) { actionManager.startLaunch(); setPathState(101); } break;
//            case 101: if (!actionManager.isBusy()) advanceToNextCommand(); break;
//
//            // --- Gate Hitting Sub-States ---
//            case 200: if (!follower.isBusy()) { follower.followPath(new Path(new BezierLine(follower.getPose(), gateTriggerPose))); setPathState(201); } break;
//            case 201: if (!follower.isBusy()) { follower.followPath(new Path(new BezierLine(follower.getPose(), gateApproachPose))); setPathState(202); } break;
//            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;
//
//            // --- Intake Cycle Sub-States ---
//            case 300: // Start of the intake dance
//                intakeCycleBallCount = 0; // Reset ball count
//                setPathState(301);
//                break;
//            case 301: // Set diverter for the current ball
//                if (!actionManager.isBusy()) {
//                    char expectedColor = intakeColorOrder.get(intakeCycleBallCount);
//                    if (expectedColor == 'P') actionManager.setDiverterPurple(); else actionManager.setDiverterGreen();
//                    setPathState(302);
//                }
//                break;
//            case 302: // Start intake
//                actionManager.toggleIntake();
//                setPathState(303);
//                break;
//            case 303: // Wait for intake to finish
//                if (!actionManager.isBusy()) {
//                    intakeCycleBallCount++;
//                    if (intakeCycleBallCount >= 3) { // Finished all 3 balls
//                        advanceToNextCommand();
//                    } else { // Move to the next ball
//                        double offset = (alliance == Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
//                        Pose nextBallPose = follower.getPose().plus(new Pose(0, offset, 0));
//                        follower.followPath(new Path(new BezierLine(follower.getPose(), nextBallPose)));
//                        setPathState(301); // Go back to set diverter for the next ball
//                    }
//                }
//                break;
//
//            case -1: // DONE
//            default: follower.breakFollowing(); break;
//        }
//    }
//
//    private void executeCommand(AutoCommand command) {
//        switch (command) {
//            case GO_TO_FRONT_SPIKE:
//                currentSpikeContext = SpikeLocation.FRONT;
//                follower.followPath(new Path(new BezierLine(follower.getPose(), frontSpike)));
//                setPathState(2);
//                break;
//            case GO_TO_MIDDLE_SPIKE:
//                currentSpikeContext = SpikeLocation.MIDDLE;
//                follower.followPath(new Path(new BezierLine(follower.getPose(), middleSpike)));
//                setPathState(2);
//                break;
//            case GO_TO_BACK_SPIKE:
//                currentSpikeContext = SpikeLocation.BACK;
//                follower.followPath(new Path(new BezierLine(follower.getPose(), backSpike)));
//                setPathState(2);
//                break;
//            case INTAKE_CYCLE:
//                // Set the correct color order based on which spike we think we're at.
//                if (currentSpikeContext == SpikeLocation.FRONT) intakeColorOrder = SPIKE_FRONT_COLORS;
//                else if (currentSpikeContext == SpikeLocation.MIDDLE) intakeColorOrder = SPIKE_MIDDLE_COLORS;
//                else intakeColorOrder = SPIKE_BACK_COLORS; // Default to back
//                setPathState(300); // Start the intake sub-state machine
//                break;
//            case SCORE:
//                follower.followPath(new Path(new BezierLine(follower.getPose(), scorePose)));
//                setPathState(100);
//                break;
//            case HIT_GATE:
//                follower.followPath(new Path(new BezierLine(follower.getPose(), gateApproachPose)));
//                setPathState(200);
//                break;
//            case PARK:
//                follower.followPath(new Path(new BezierLine(follower.getPose(), parkPose)));
//                setPathState(2);
//                break;
//        }
//    }
//
//    private void advanceToNextCommand() {
//        currentCommandIndex++;
//        setPathState(1); // Go execute the next command (or finish if done)
//    }
//}
