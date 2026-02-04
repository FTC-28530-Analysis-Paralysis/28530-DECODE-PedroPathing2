package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionManager {

    private final RobotHardwareContainer robot;
    private final LauncherHardware launcher;
    private final FeederHardware feeder;
    private final IntakeHardware intake;
    private final ColorDiverterHardware colorDiverter;
    private final ElapsedTime actionTimer = new ElapsedTime();

    private static final double FEED_TIME_SECONDS = 1.2; // Duration to runLeft the feeder for a single shot
    private static final double FIRE_COMMAND_TIMEOUT_SECONDS = 0.75; // Time to wait for launcher to be ready

    public enum ActionState {
        IDLE,
        LEFT_WAITING_FOR_LAUNCHER_SPEED,
        RIGHT_WAITING_FOR_LAUNCHER_SPEED,
        LEFT_LAUNCHING_SPINUP,
        RIGHT_LAUNCHING_SPINUP,
        LAUNCHING_FIRE,
        WAITING_FOR_LAUNCHER_SPEED, // New state for fire-with-timeout
        FIRING_ARTIFACT,
        REVERSING,
        ACTION_COMPLETE
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robotContainer) {
        this.robot = robotContainer;
        this.launcher = robot.launcher;
        this.feeder = robot.feeder;
        this.intake = robot.intake;
        this.colorDiverter = robot.colorDiverter; // This can be null
    }

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE: case REVERSING:
                break; // No timed logic in these states

            case LEFT_WAITING_FOR_LAUNCHER_SPEED:
                if (launcher.isAtTargetSpeed()) {
                    // Launcher is ready, fire now!
                    currentState = ActionState.FIRING_ARTIFACT;
                    feeder.runLeft();
                    intake.run();
                    actionTimer.reset();
                } else if (actionTimer.seconds() > FIRE_COMMAND_TIMEOUT_SECONDS) {
                    // Timeout exceeded, cancel the fire command.
                    currentState = ActionState.IDLE;
                }
                break;
            case RIGHT_WAITING_FOR_LAUNCHER_SPEED:
                if (launcher.isAtTargetSpeed()) {
                    // Launcher is ready, fire now!
                    currentState = ActionState.FIRING_ARTIFACT;
                    feeder.runRight();
                    intake.run();
                    actionTimer.reset();
                } else if (actionTimer.seconds() > FIRE_COMMAND_TIMEOUT_SECONDS) {
                    // Timeout exceeded, cancel the fire command.
                    currentState = ActionState.IDLE;
                }
                break;

            case LEFT_LAUNCHING_SPINUP:
                if (launcher.isAtTargetSpeed()) {
                    feeder.runLeft();
                    intake.run();
                    actionTimer.reset();
                    currentState = ActionState.LAUNCHING_FIRE;
                }
                break;
            case RIGHT_LAUNCHING_SPINUP:
                if (launcher.isAtTargetSpeed()) {
                    feeder.runRight();
                    intake.run();
                    actionTimer.reset();
                    currentState = ActionState.LAUNCHING_FIRE;
                }
                break;

            case LAUNCHING_FIRE: // This is part of the auto-launch sequence
                if (actionTimer.seconds() > FEED_TIME_SECONDS) {
                    feeder.stop();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;

            case FIRING_ARTIFACT: // The new state for TeleOp artifact firing
                if (actionTimer.seconds() > FEED_TIME_SECONDS) {
                    feeder.stop();
                    currentState = ActionState.IDLE; // Return to idle, ready for next command
                }
                break;
        }
    }

    // ----- Public Methods to Trigger Actions -----
    public ActionState getCurrentState() {
        return currentState;
    }

    /** This is a full launch sequence, good for autonomous. It spins up and then fires. */
    public void startLeftLaunch() {
        if (isBusy()) return;
        currentState = ActionState.LEFT_LAUNCHING_SPINUP;
        launcher.start(); // Corrected method call
    }
    /** This is a full launch sequence, good for autonomous. It spins up and then fires. */
    public void startRightLaunch() {
        if (isBusy()) return;
        currentState = ActionState.RIGHT_LAUNCHING_SPINUP;
        launcher.start(); // Corrected method call
    }

    /**
     * This is for TeleOp. If the launcher is at speed, it fires an artifact immediately.
     * If not, it waits for a short period. If the launcher gets to speed in time, it fires.
     * Otherwise, the command is cancelled.
     */
    public void fireLeftArtifactWhenReady() {
        if (isBusy()) return; // Eject if another action is in progress

        if (launcher.isAtTargetSpeed()) {
            // Already at speed, fire immediately.
            currentState = ActionState.FIRING_ARTIFACT;
            feeder.runLeft();
            intake.run();
            actionTimer.reset();
        } else {
            // Not at speed, so we enter the waiting state with a timeout.
            currentState = ActionState.WAITING_FOR_LAUNCHER_SPEED;
            actionTimer.reset();
        }
    }

    /**
     * This is for TeleOp. If the launcher is at speed, it fires an artifact immediately.
     * If not, it waits for a short period. If the launcher gets to speed in time, it fires.
     * Otherwise, the command is cancelled.
     */
    public void fireRightArtifactWhenReady() {
        if (isBusy()) return; // Eject if another action is in progress

        if (launcher.isAtTargetSpeed()) {
            // Already at speed, fire immediately.
            currentState = ActionState.FIRING_ARTIFACT;
            feeder.runRight();
            intake.run();
            actionTimer.reset();
        } else {
            // Not at speed, so we enter the waiting state with a timeout.
            currentState = ActionState.WAITING_FOR_LAUNCHER_SPEED;
            actionTimer.reset();
        }
    }


    public void reverseAll() {
        if (currentState == ActionState.REVERSING) return; // Prevent re-triggering
        currentState = ActionState.REVERSING;
        intake.reverse();
        launcher.reverse();
        feeder.reverse();
    }

    public void setDiverterPurple() {
        if (colorDiverter != null) {
            colorDiverter.setPosition(ColorDiverterHardware.GatePosition.PURPLE);
        }
    }

    public void setDiverterGreen() {
        if (colorDiverter != null) {
            colorDiverter.setPosition(ColorDiverterHardware.GatePosition.GREEN);
        }
    }

    public boolean isBusy() {
        return currentState != ActionState.IDLE && currentState != ActionState.ACTION_COMPLETE;
    }

    public void stopAll() {
        intake.stop();
        launcher.stop();
        feeder.stop();
        currentState = ActionState.IDLE;
    }

    public ActionState getActionState(){
        return currentState;
    }
}
