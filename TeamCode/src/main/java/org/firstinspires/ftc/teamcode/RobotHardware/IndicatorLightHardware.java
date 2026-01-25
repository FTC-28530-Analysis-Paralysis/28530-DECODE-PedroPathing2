package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IndicatorLightHardware {

    private Servo indicatorLight = null;

    // Color constants
    public static final double COLOR_GREEN = 0.500;
    public static final double COLOR_RED = 0.277;
    public static final double COLOR_BLUE = 0.611;
    public static final double COLOR_OFF = 0.0;

    // Blinking constants
    private static final long BLINK_HALF_PERIOD_MS = 500; // 0.5 seconds, for a 1Hz blink frequency

    // State management
    private enum LightState { OFF, RED, GREEN, BLUE, BLINKING }
    private LightState currentState = LightState.OFF;
    private LightState returnState = LightState.OFF;
    private double blinkColor = COLOR_OFF;

    private final ElapsedTime blinkTimer = new ElapsedTime();
    private double blinkDuration = 0;
    private boolean isBlinking = false;

    public void init(HardwareMap hardwareMap) {
        try {
            indicatorLight = hardwareMap.get(Servo.class, "indicator_light");
        } catch (Exception e) {
            indicatorLight = null;
        }
        setOff();
    }

    public void update() {
        if (isBlinking) {
            // Simple blink effect: toggle on and off based on the half period
            if (blinkTimer.milliseconds() < blinkDuration) {
                if (((int)(blinkTimer.milliseconds() / BLINK_HALF_PERIOD_MS)) % 2 == 0) {
                    setPosition(blinkColor);
                } else {
                    setPosition(COLOR_OFF);
                }
            } else {
                // Blinking finished, revert to the previous state
                isBlinking = false;
                currentState = returnState;
                applyState();
            }
        }
    }

    public void blink(double color, double seconds) {
        if (!isBlinking) {
            returnState = currentState;
        }
        isBlinking = true;
        blinkColor = color;
        blinkDuration = seconds * 1000; // Convert to milliseconds
        blinkTimer.reset();
    }

    public void setGreen() {
        if (!isBlinking) {
            currentState = LightState.GREEN;
            applyState();
        }
    }

    public void setRed() {
        if (!isBlinking) {
            currentState = LightState.RED;
            applyState();
        }
    }

    public void setBlue() {
        if (!isBlinking) {
            currentState = LightState.BLUE;
            applyState();
        }
    }

    public void setOff() {
        if (!isBlinking) {
            currentState = LightState.OFF;
            applyState();
        }
    }

    private void applyState() {
        switch (currentState) {
            case GREEN:
                setPosition(COLOR_GREEN);
                break;
            case RED:
                setPosition(COLOR_RED);
                break;
            case BLUE:
                setPosition(COLOR_BLUE);
                break;
            case OFF:
            default:
                setPosition(COLOR_OFF);
                break;
        }
    }

    private void setPosition(double position) {
        if (indicatorLight != null) {
            indicatorLight.setPosition(position);
        }
    }
}
