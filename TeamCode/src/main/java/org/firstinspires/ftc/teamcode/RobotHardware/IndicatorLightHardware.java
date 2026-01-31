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
    public static final double COLOR_ORANGE = 0.333;
    public static final double COLOR_OFF = 0.0;

    // Blinking constants
    private static final long BLINK_HALF_PERIOD_MS = 500; // 0.5 seconds, for a 1Hz blink frequency

    // State management
    private enum LightState { STATIC, BLINKING }
    private LightState currentState = LightState.STATIC;
    private double staticColor = COLOR_OFF; // The color to show when not blinking
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
        setStaticColor(COLOR_OFF); // Start with the light off
    }

    public void update() {
        if (isBlinking) {
            // If the blink duration has expired...
            if (blinkTimer.milliseconds() >= blinkDuration) {
                isBlinking = false;
                // ...revert to the last set static color.
                setValue(staticColor);
            } else {
                // Otherwise, continue blinking by toggling between the blink color and OFF.
                if (((long)(blinkTimer.milliseconds() / BLINK_HALF_PERIOD_MS)) % 2 == 0) {
                    setValue(blinkColor);
                } else {
                    setValue(COLOR_OFF);
                }
            }
        }
    }

    /**
     * Blinks a specific color for a set number of seconds, then returns to the previous static color.
     * @param color The color to blink (e.g., COLOR_RED).
     * @param seconds The duration of the blinking.
     */
    public void blink(double color, double seconds) {
        // If not already blinking, we store the current static color so we can return to it.
        if (!isBlinking) {
            // The return state is implicitly the 'staticColor' variable.
        }
        isBlinking = true;
        blinkColor = color;
        blinkDuration = seconds * 1000; // Convert to milliseconds
        blinkTimer.reset();
    }

    /**
     * Sets the indicator to a solid, non-blinking color.
     * This will also be the color the light returns to after a blink is finished.
     * @param color The desired static color (e.g., COLOR_BLUE).
     */
    public void setStaticColor(double color) {
        this.staticColor = color;
        // If we aren't in the middle of a blink, apply the color immediately.
        if (!isBlinking) {
            currentState = LightState.STATIC;
            setValue(staticColor);
        }
    }

    // --- Convenience methods (Optional, but good for compatibility) ---

    public void setGreen() {
        setStaticColor(COLOR_GREEN);
    }

    public void setRed() {
        setStaticColor(COLOR_RED);
    }

    public void setBlue() {
        setStaticColor(COLOR_BLUE);
    }

    public void setOff() {
        setStaticColor(COLOR_OFF);
    }

    /**
     * Directly sets the light value. This is the single point of control.
     * The GoBilda Indicator light takes a value from 0 to 1, treating it like a servo by setting a position.
     */
    private void setValue(double value) {
        if (indicatorLight != null) {
            indicatorLight.setPosition(value);
        }
    }
}
