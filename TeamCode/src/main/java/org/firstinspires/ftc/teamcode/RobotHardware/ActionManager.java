package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionManager {

    private final RobotHardwareContainer robot;
    private final IntakeHardware intake;
    private final LauncherHardware launcher;
    private final FeederHardware transfer;
    private final ColorDiverterHardware colorDiverter;
    private final ElapsedTime actionTimer = new ElapsedTime();

    private static final double FEED_TIME_SECONDS = 0.8; // Duration to run the feeder for a single shot
    private static final double FIRE_COMMAND_TIMEOUT_SECONDS = 0.75; // Time to wait for launcher to be ready

    public enum ActionState {
        IDLE,
        INTAKING,
        LAUNCHING_SPINUP,
        LAUNCHING_FIRE,
        WAITING_FOR_LAUNCHER_SPEED, // New state for fire-with-timeout
        FIRING_ARTIFACT,
        REVERSING,
        ACTION_COMPLETE
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robotContainer) {
        this.robot = robotContainer;
        this.intake = robot.intake;
        this.launcher = robot.launcher;
        this.transfer = robot.transfer;
        this.colorDiverter = robot.colorDiverter; // This can be null
    }

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE: case REVERSING:
                break; // No timed logic in these states

            case INTAKING:
                // This action is manually stopped
                break;

            case WAITING_FOR_LAUNCHER_SPEED:
                if (launcher.isAtTargetSpeed()) {
                    // Launcher is ready, fire now!
                    currentState = ActionState.FIRING_ARTIFACT;
                    transfer.run();
                    actionTimer.reset();
                } else if (actionTimer.seconds() > FIRE_COMMAND_TIMEOUT_SECONDS) {
                    // Timeout exceeded, cancel the fire command.
                    currentState = ActionState.IDLE;
                }
                break;

            case LAUNCHING_SPINUP:
                if (launcher.isAtTargetSpeed()) {
                    transfer.run();
                    actionTimer.reset();
                    currentState = ActionState.LAUNCHING_FIRE;
                }
                break;

            case LAUNCHING_FIRE: // This is part of the auto-launch sequence
                if (actionTimer.seconds() > FEED_TIME_SECONDS) {
                    stopAll();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;

            case FIRING_ARTIFACT: // The new state for TeleOp artifact firing
                if (actionTimer.seconds() > FEED_TIME_SECONDS) {
                    transfer.stop();
                    currentState = ActionState.IDLE; // Return to idle, ready for next command
                }
                break;
        }
    }

    // ----- Public Methods to Trigger Actions -----
    public ActionState getCurrentState() {
        return currentState;
    }

    public void startIntake() {
        if (isBusy()) return;
        currentState = ActionState.INTAKING;
        intake.run();
        actionTimer.reset();
    }

    /** This is a full launch sequence, good for autonomous. It spins up and then fires. */
    public void startLaunch() {
        if (isBusy()) return;
        currentState = ActionState.LAUNCHING_SPINUP;
        launcher.start(); // Corrected method call
    }

    /**
     * This is for TeleOp. If the launcher is at speed, it fires an artifact immediately.
     * If not, it waits for a short period. If the launcher gets to speed in time, it fires.
     * Otherwise, the command is cancelled.
     */
    public void fireArtifactWhenReady() {
        if (isBusy()) return; // Eject if another action is in progress

        if (launcher.isAtTargetSpeed()) {
            // Already at speed, fire immediately.
            currentState = ActionState.FIRING_ARTIFACT;
            transfer.run();
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
        transfer.reverse();
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
        transfer.stop();
        currentState = ActionState.IDLE;
    }
}
