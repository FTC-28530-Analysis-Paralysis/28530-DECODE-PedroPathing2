package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A highly configurable, playlist-based Autonomous OpMode with combined actions.
 *
 * This OpMode allows the drive team to build a sequence of autonomous actions. Key commands
 * are combined to make playlist creation faster and more intuitive. For example, selecting
 * a spike mark command will automatically handle both moving to the spike and intaking.
 *
 * --- KEY FEATURES ---
 * - Combined Actions: Complex multi-step actions (e.g., "go to spike and intake all three")
 *   are condensed into single, selectable commands.
 * - State Persistence: Saves the robot's final pose and alliance to the static `GameState`,
 *   allowing `BozemanTeleop` to seamlessly take over with full field awareness.
 *
 * --- PLAYLIST BUILDER CONTROLS (GAMEPAD 1 - INIT) ---
 * D-Pad Up/Down:   Select Alliance (Blue/Red)
 * Left/Right Bumper: Select Starting Position (Front/Back)
 * D-Pad Left/Right:  Scroll through the list of available auto commands.
 * A Button:          Add the currently selected command to the playlist.
 * B Button:          Remove the last command from the playlist.
 * Y Button:          Clear the entire playlist.
 * X Button:          Finalize/Lock the playlist to prevent accidental changes.
 *
 */
@Autonomous(name = "Bozeman Auto", group = "01 Bozeman", preselectTeleOp = "BozemanTeleop")
public class BozemanAuto extends OpMode {

    // ========== OPMODE CONFIGURATION (SET DURING INIT) ========== //

    /** Represents the two starting positions on the field. */
    private enum StartPosition { FRONT, BACK }

    /**
     * These are the high-level commands the drive team can choose from to build their playlist.
     * Each command represents a complete action, like moving to a spike and intaking.
     */
    private enum AutoCommand {
        SCORE_ALL_THREE_CLOSE,
        SCORE_ALL_THREE_FAR,
        SPIKE_FRONT_AND_INTAKE,
        SPIKE_MIDDLE_AND_INTAKE,
        SPIKE_BACK_AND_INTAKE,
        HIT_GATE,
        PARK
    }

    /** A helper enum to keep track of which spike mark we are currently at. */
    private enum SpikeLocation { NONE, FRONT, MIDDLE, BACK }

    // --- Default selections, can be changed by the driver during init ---
    private GameState.Alliance alliance = GameState.Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    private SpikeLocation currentSpikeContext = SpikeLocation.NONE;

    // --- Playlist variables ---
    private ArrayList<AutoCommand> autoCommands = new ArrayList<>(); // The list of commands to run
    private int commandMenuIndex = 0;           // The currently selected command in the D-Pad menu
    private int currentCommandIndex = 0;        // The index of the command currently being executed
    private boolean isPlaylistFinalized = false; // A flag to lock the playlist from accidental changes

    // ========== OPMODE CORE MEMBERS ========== //
    private PinpointLocalizer localizer;        // The combined localizer for localization
    private Follower follower;                  // The Pedro Pathing follower for path execution
    private ElapsedTime timer = new ElapsedTime(); // A general-purpose timer
    private RobotHardwareContainer robot;       // Container for all robot hardware
    private ActionManager actionManager;        // Manages complex, timed actions

    // === PATHING & STATE MANAGEMENT === //
    private Pose startPose, scoreClosePose, scoreFarPose, parkPose;
    private Pose frontSpike, middleSpike, backSpike;
    private Pose gateApproachPose, gateTriggerPose;
    private int pathState = 0;                  // The current step in our state machine

    // --- Cycle counters, used to track progress through multi-step actions ---
    private int intakeCycleArtifactCount = 0; // How many Artifacts we've collected in the current intake cycle
    private int scoreCycleArtifactCount = 0;  // How many Artifacts we've launched in the current scoring cycle
    private List<Character> intakeColorOrder = new ArrayList<>(); // The expected order of Artifact colors
    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
    private final double INTAKE_OFFSET_DISTANCE = 6.0; // Distance to move between Artifacts in a stack

    /**
     * Code to run ONCE when the driver hits INIT.
     * Initializes hardware and displays instructions on the Driver Station.
     */
    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        localizer = new PinpointLocalizer(hardwareMap, new PinpointConstants());
        follower = Constants.createFollower(hardwareMap, localizer);

        telemetry.addLine("--- Playlist Autonomous Builder ---");
        telemetry.addLine("X: Lock Playlist | A: Add | B: Remove | Y: Clear");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY during the init phase.
     * This is where the drive team builds their autonomous playlist.
     */
    @Override
    public void init_loop() {
        // Allow the user to lock the playlist to prevent accidental changes
        if (gamepad1.xWasPressed()) {
            isPlaylistFinalized = !isPlaylistFinalized;
        }

        // Only allow modifications if the playlist is not finalized
        if (!isPlaylistFinalized) {
            // --- Configure Alliance and Start Position ---
            if (gamepad1.dpadUpWasPressed()) alliance = GameState.Alliance.BLUE;
            if (gamepad1.dpadDownWasPressed()) alliance = GameState.Alliance.RED;

            if (gamepad1.leftBumperWasPressed()) startPosition = StartPosition.FRONT;
            if (gamepad1.rightBumperWasPressed()) startPosition = StartPosition.BACK;

            // --- Build the Playlist ---
            AutoCommand[] allCommands = AutoCommand.values();
            // Scroll through commands with D-Pad
            if (gamepad1.dpadRightWasPressed()) commandMenuIndex = (commandMenuIndex + 1) % allCommands.length;
            if (gamepad1.dpadLeftWasPressed()) commandMenuIndex = (commandMenuIndex - 1 + allCommands.length) % allCommands.length;

            // Add, remove, or clear commands
            if (gamepad1.aWasPressed()) autoCommands.add(allCommands[commandMenuIndex]);
            if (gamepad1.yWasPressed()) autoCommands.clear();
            if (gamepad1.bWasPressed() && !autoCommands.isEmpty()) {
                autoCommands.remove(autoCommands.size() - 1);
            }
        }

        // --- Display Selections on Driver Station ---
        if (isPlaylistFinalized) {
            telemetry.addLine("*** PLAYLIST FINALIZED (Press X to Unlock) ***");
        } else {
            telemetry.addData("--> Selected Command", AutoCommand.values()[commandMenuIndex]);
        }
        telemetry.addData("Alliance", alliance).addData("Start", startPosition);
        telemetry.addLine("\nCurrent Playlist:");
        for (int i = 0; i < autoCommands.size(); i++) {
            telemetry.addLine((i+1) + ". " + autoCommands.get(i));
        }
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits PLAY.
     * It calculates all necessary poses and starts the state machine.
     */
    @Override
    public void start() {
        // Store the selected alliance so TeleOp can use it.
        GameState.alliance = this.alliance;

        // Spin up the intake and launcher at the start of autonomous
        actionManager.startIntake();
        robot.launcher.start();

        calculatePoses(); // Determine all field coordinates based on alliance/start pos
        follower.setStartingPose(startPose);
        currentCommandIndex = 0;
        if (!autoCommands.isEmpty()) {
            setPathState(1); // Start the first command
        } else {
            setPathState(-1); // No commands, end immediately
        }
        follower.update();
    }

    /**
     * Code to run REPEATEDLY during the autonomous period.
     * This is the main loop that drives the state machine.
     */
    @Override
    public void loop() {
        follower.update();
        actionManager.update();
        robot.launcher.update(follower.getPose()); // This is crucial for dynamic speed in Auto
        updatePath(); // The heart of the autonomous logic

        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    /**
     * Code to run ONCE after the OpMode ends.
     * Saves the robot's final pose to GameState for a seamless transition to TeleOp.
     */
    @Override
    public void stop() {
        if (follower.getPose() != null) {
            GameState.currentPose = follower.getPose();
        }
    }

    /**
     * Calculates all the target positions on the field based on the selected alliance and
     * starting position. This keeps all coordinate logic in one place.
     */
    private void calculatePoses() {
        if (alliance == GameState.Alliance.BLUE) {
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
            scoreClosePose = FieldPosePresets.BLUE_SCORE_CLOSE_TO_GOAL;
            scoreFarPose = FieldPosePresets.BLUE_SCORE_FAR_FROM_GOAL;
            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
            frontSpike = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.BLUE_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.BLUE_GATE_TRIGGER;
        } else { // RED Alliance
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
            scoreClosePose = FieldPosePresets.RED_SCORE_CLOSE_TO_GOAL;
            scoreFarPose = FieldPosePresets.RED_SCORE_FAR_FROM_GOAL;
            parkPose = FieldPosePresets.RED_AUTO_PARK;
            frontSpike = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.RED_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.RED_GATE_TRIGGER;
        }
    }

    /**
     * A helper method to change the state of our state machine.
     * @param newState The new state to transition to.
     */
    private void setPathState(int newState) {
        pathState = newState;
        timer.reset();
    }

    /**
     * This is the main state machine that runs our autonomous routine.
     * It executes commands from the playlist one by one, handling all the complex
     * multi-step actions.
     */
    private void updatePath() {
        if (currentCommandIndex >= autoCommands.size()) {
            setPathState(-1); // Done with playlist
            return;
        }

        switch (pathState) {
            case 0: break; // IDLE

            case 1: executeCommand(autoCommands.get(currentCommandIndex)); break;

            case 2: // A simple waiting state for basic paths or actions to complete.
                if (!follower.isBusy() && !actionManager.isBusy()) advanceToNextCommand();
                break;

            // --- Gate Hitting Sub-States ---
            case 200: if (!follower.isBusy()) { follower.followPath(new Path(new BezierLine(follower.getPose(), gateTriggerPose))); setPathState(201); } break;
            case 201: if (!follower.isBusy()) { follower.followPath(new Path(new BezierLine(follower.getPose(), gateApproachPose))); setPathState(202); } break;
            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;

            // --- Intake Cycle Sub-States (now triggered by a SPIKE_*_AND_INTAKE command) ---
            case 300: // Wait for arrival at the spike mark.
                if (!follower.isBusy()) {
                    intakeCycleArtifactCount = 0; // Reset counter for the new cycle
                    setPathState(301);
                }
                break;
            case 301: // Set the diverter to the correct color for the first artifact.
                if (!actionManager.isBusy()) {
                    char expectedColor = intakeColorOrder.get(intakeCycleArtifactCount);
                    if (expectedColor == 'P') actionManager.setDiverterPurple(); else actionManager.setDiverterGreen();
                    setPathState(302);
                }
                break;
            case 302: // Start the intake motor.
                actionManager.startIntake();
                setPathState(303);
                break;
            case 303: // Wait for the intake action to complete.
                if (!actionManager.isBusy()) {
                    intakeCycleArtifactCount++;
                    if (intakeCycleArtifactCount >= 3) { // If we have all 3 artifacts...
                        advanceToNextCommand();      // ...then this command is done.
                    } else { // Otherwise, move to the next artifact in the stack.
                        double offset = (alliance == GameState.Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
                        Pose nextArtifactPose = follower.getPose().plus(new Pose(offset, 0, 0));
                        follower.followPath(new Path(new BezierLine(follower.getPose(), nextArtifactPose)));
                        setPathState(301); // Go back and set diverter for the next artifact.
                    }
                }
                break;

            // --- Scoring Cycle Sub-States ---
            case 400: // Wait for arrival at the scoring position.
                if (!follower.isBusy()) {
                    scoreCycleArtifactCount = 0; // Reset counter
                    setPathState(401);
                }
                break;
            case 401: // This is a loop that fires all three artifacts.
                if (!actionManager.isBusy()) { // Wait for the previous shot to complete.
                    if (scoreCycleArtifactCount < 3) {
                        actionManager.startLaunch(); // This ActionManager method fires one artifact.
                        scoreCycleArtifactCount++;
                    } else {
                        advanceToNextCommand(); // All three have been fired, move on.
                    }
                }
                break;

            case -1: // DONE
            default: follower.breakFollowing(); break;
        }
    }

    /**
     * This method translates a high-level AutoCommand from the playlist into a concrete
     * action for the robot, like following a path or entering a state machine sequence.
     * @param command The command to execute.
     */
    private void executeCommand(AutoCommand command) {
        switch (command) {
            case SPIKE_FRONT_AND_INTAKE:
                currentSpikeContext = SpikeLocation.FRONT;
                intakeColorOrder = SPIKE_FRONT_COLORS;
                follower.followPath(new Path(new BezierLine(follower.getPose(), frontSpike)));
                setPathState(300); // Enter the intake cycle state machine
                break;
            case SPIKE_MIDDLE_AND_INTAKE:
                currentSpikeContext = SpikeLocation.MIDDLE;
                intakeColorOrder = SPIKE_MIDDLE_COLORS;
                follower.followPath(new Path(new BezierLine(follower.getPose(), middleSpike)));
                setPathState(300);
                break;
            case SPIKE_BACK_AND_INTAKE:
                currentSpikeContext = SpikeLocation.BACK;
                intakeColorOrder = SPIKE_BACK_COLORS;
                follower.followPath(new Path(new BezierLine(follower.getPose(), backSpike)));
                setPathState(300);
                break;
            case SCORE_ALL_THREE_CLOSE:
                follower.followPath(new Path(new BezierLine(follower.getPose(), scoreClosePose)));
                setPathState(400); // Enter the scoring cycle state machine
                break;
            case SCORE_ALL_THREE_FAR:
                follower.followPath(new Path(new BezierLine(follower.getPose(), scoreFarPose)));
                setPathState(400); // Enter the scoring cycle state machine
                break;
            case HIT_GATE:
                follower.followPath(new Path(new BezierLine(follower.getPose(), gateApproachPose)));
                setPathState(200); // Enter the gate hitting state machine
                break;
            case PARK:
                follower.followPath(new Path(new BezierLine(follower.getPose(), parkPose)));
                setPathState(2); // Use the simple "wait" state
                break;
        }
    }

    /**
     * A helper method to advance to the next command in the playlist.
     */
    private void advanceToNextCommand() {
        currentCommandIndex++;
        setPathState(1); // Go execute the next command (or finish if done)
    }
}
