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
import org.firstinspires.ftc.teamcode.RobotHardware.IndicatorLightHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
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
 * --- ALLIANCE & STARTING POSITION SELECTION
 * D-Pad Left/Right:  Select Alliance (Blue/Red)
 * D-Pad Up/Down:     Select Starting Position (Front/Back)
 *
 * --- PLAYLIST BUILDER CONTROLS (GAMEPAD 1 - INIT) ---
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
    private enum SpikeLocation { FRONT, MIDDLE, BACK }
    public enum AutoCommand {
        SPIKE_FRONT_AND_INTAKE, SPIKE_MIDDLE_AND_INTAKE, SPIKE_BACK_AND_INTAKE,
        SCORE_ALL_THREE_CLOSE, SCORE_ALL_THREE_FAR, HIT_GATE, PARK
    }

    // ========== AUTONOMOUS PLAYLIST BUILDER MEMBERS ========== //
    private GameState.Alliance alliance = GameState.Alliance.BLUE; // Default alliance
    private StartPosition startPosition = StartPosition.FRONT; // Default start position
    private SpikeLocation currentSpikeContext;  // Which spike we are currently targeting
    private ArrayList<AutoCommand> autoCommands = new ArrayList<>(); // The list of commands to execute
    private int commandMenuIndex = 0;           // The currently selected command in the D-Pad menu
    private int currentCommandIndex = 0;        // The index of the command currently being executed
    private boolean isStartPoseSelected = false; // A flag to finish alliance and starting location selection before moving onto playlist building
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
    private List<Character> scoreOrder = new ArrayList<>(); // The order to launch artifacts (L/R)
    private final double INTAKE_OFFSET_DISTANCE = 6.0; // Distance to move between Artifacts in a stack

    /**
     * Code to runLeft ONCE when the driver hits INIT.
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
     * Code to runLeft REPEATEDLY during the init phase.
     * This is where the drive team builds their autonomous playlist.
     */
    @Override
    public void init_loop() {
        if (!isStartPoseSelected) {
            selectStartingLocation();
        } else playlistBuilder();
    }

    /**
     * Code to runLeft ONCE when the driver hits PLAY.
     * It calculates all necessary poses and starts the state machine.
     */
    @Override
    public void start() {
        // Store the selected alliance so TeleOp can use it.
        GameState.alliance = this.alliance;

        // Spin up the intake and launcher at the start of autonomous
        robot.intake.run();
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
     * Code to runLeft REPEATEDLY during the autonomous period.
     * This is the main loop that drives the state machine.
     */
    @Override
    public void loop() {
        follower.update();
        actionManager.update();
        robot.launcher.update(follower.getPose()); // This is crucial for dynamic speed in Auto
        updatePath(); // The heart of the autonomous logic
        follower.update();

        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionManager.getCurrentState());
        telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * Code to runLeft ONCE after the OpMode ends.
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
            case 200: if (!follower.isBusy()) { followerPathBuilder(gateTriggerPose); setPathState(201); } break;
            case 201: if (!follower.isBusy()) { followerPathBuilder(gateApproachPose); setPathState(202); } break;
            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;

            // --- Intake Cycle Sub-States ---
            case 300: // Wait for arrival at the first spike mark.
                if (!follower.isBusy()) {
                    intakeCycleArtifactCount = 0; // Reset for the new cycle
                    setPathState(301);
                }
                break;

            case 301: // Set diverter and prepare for timed intake.
                if (intakeCycleArtifactCount == 0) {
                    actionManager.setDiverterGreen();
                } else if (intakeCycleArtifactCount == 1) {
                    actionManager.setDiverterPurple();
                }
                robot.intake.run();
                setPathState(302); // Move to the waiting state, which resets the timer.
                break;

            case 302: // Wait for 1 second for the artifact to be fully captured.
                if (timer.seconds() > 1.0) {
                    intakeCycleArtifactCount++;
                    setPathState(303);
                }
                break;

            case 303: // Check if we're done, or move to the next artifact.
                if (intakeCycleArtifactCount >= 3) {
                    advanceToNextCommand(); // We have all three, command is done.
                } else {
                    double offset = (alliance == GameState.Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
                    Pose nextArtifactPose = follower.getPose().plus(new Pose(offset, 0, 0));
                    followerPathBuilder(nextArtifactPose);
                    setPathState(304); // Go to a state that waits for the move to finish.
                }
                break;

            case 304: // Wait for the robot to finish moving to the next artifact.
                if (!follower.isBusy()) {
                    setPathState(301); // Once there, go back to intake the next one.
                }
                break;

            // --- Scoring Cycle Sub-States ---
            case 400: // Wait for arrival at the scoring position.
                if (!follower.isBusy()) {
                    scoreCycleArtifactCount = 0; // Reset counter
                    setPathState(401);
                }
                break;
            case 401: // Fire all three artifacts in the specified order.
                if (!actionManager.isBusy()) {
                    if (scoreCycleArtifactCount < scoreOrder.size()) {
                        char launchSide = scoreOrder.get(scoreCycleArtifactCount);
                        if (launchSide == 'L') {
                            actionManager.startLeftLaunch();
                        } else { // Default to right for any other character
                            actionManager.startRightLaunch();
                        }
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

    private void executeCommand(AutoCommand command) {
        switch (command) {
            case SPIKE_FRONT_AND_INTAKE:
                currentSpikeContext = SpikeLocation.FRONT;
                followerPathBuilder(frontSpike);
                setPathState(300); // Enter the intake cycle state machine
                break;
            case SPIKE_MIDDLE_AND_INTAKE:
                currentSpikeContext = SpikeLocation.MIDDLE;
                followerPathBuilder(middleSpike);
                setPathState(300);
                break;
            case SPIKE_BACK_AND_INTAKE:
                currentSpikeContext = SpikeLocation.BACK;
                followerPathBuilder(backSpike);
                setPathState(300);
                break;
            case SCORE_ALL_THREE_CLOSE:
                scoreOrder = Arrays.asList('L', 'L', 'R'); // Set launch order: 2 Left, 1 Right
                followerPathBuilder(scoreClosePose);
                setPathState(400); // Enter the scoring cycle state machine
                break;
            case SCORE_ALL_THREE_FAR:
                scoreOrder = Arrays.asList('L', 'L', 'R'); // Set launch order: 2 Left, 1 Right
                followerPathBuilder(scoreFarPose);
                setPathState(400); // Enter the scoring cycle state machine
                break;
            case HIT_GATE:
                followerPathBuilder(gateApproachPose);
                setPathState(200); // Enter the gate hitting state machine
                break;
            case PARK:
                followerPathBuilder(parkPose);
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

    /**
     * A helper method to build the path from the robot's current position to the next target position.
     * @param targetPose
     */
    private void followerPathBuilder(Pose targetPose){
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                .build());
    }

    private void selectStartingLocation(){
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            alliance = GameState.Alliance.BLUE;
            robot.indicatorLight.setStaticColor(IndicatorLightHardware.COLOR_BLUE);
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            alliance = GameState.Alliance.RED;
            robot.indicatorLight.setStaticColor(IndicatorLightHardware.COLOR_RED);
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up) startPosition = StartPosition.BACK;
        if (gamepad1.dpad_down || gamepad2.dpad_down) startPosition = StartPosition.FRONT;

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            isStartPoseSelected = true;
        }

        // Provide continuous feedback on the driver station
        telemetry.addLine("Blue/Red Alliance: dPad left/right");
        telemetry.addLine("Starting Position Back/Front: dPad up/down");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Start", startPosition);
        telemetry.addLine("\nPress 'A to confirm and start building playlist");
        telemetry.update();
    }

    private void playlistBuilder() {
        // Allow the user to lock the playlist to prevent accidental changes
        if (gamepad1.xWasPressed()) {
            isPlaylistFinalized = !isPlaylistFinalized;
        }

        // Only allow modifications if the playlist is not finalized
        if (!isPlaylistFinalized) {
            AutoCommand[] allCommands = AutoCommand.values();
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
            telemetry.addLine("A: add command to playlist, B: remove last command, X: finalize playlist");
            telemetry.addData("--> Selected Command", AutoCommand.values()[commandMenuIndex]);
        }
        telemetry.addData("Alliance", alliance).addData("Start", startPosition);
        telemetry.addLine("\nCurrent Playlist:");
        for (int i = 0; i < autoCommands.size(); i++) {
            telemetry.addLine((i+1) + ". " + autoCommands.get(i));
        }
        telemetry.update();
    }
}
