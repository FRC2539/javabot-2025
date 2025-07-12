package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ModeManager.ModeManager;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class LightsSubsystem extends SubsystemBase {
    public static final class LightsConstants {
        public static final int CANDLE_PORT = 13;

        public static final int SENSOR_PORT = 0;
    }

    private BooleanSupplier algaeMode = () -> false;

    private static final CANdle candle;

    private static final boolean isReal = true;

    static {
        if (RobotBase.isReal() && isReal) {
            candle = new CANdle(LightsConstants.CANDLE_PORT, "CANivore");
        } else {
            candle = null;
        }
    }

    // Team colors
    public static final Color orange = new Color(255, 25, 0);
    public static final Color black = new Color(0, 0, 0);

    // Game piece colors
    public static final Color yellow = new Color(242, 60, 0);
    public static final Color purple = new Color(184, 0, 185);

    // Indicator colors
    public static final Color white = new Color(255, 230, 220);
    public static final Color green = new Color(56, 209, 0);
    public static final Color blue = new Color(8, 32, 255);
    public static final Color red = new Color(255, 0, 0);
    public static final Color gray = new Color(75, 75, 75);

    public LightsSubsystem() {
        if (candle != null) {
            CANdleConfiguration candleConfiguration = new CANdleConfiguration();
            candleConfiguration.statusLedOffWhenActive = true;
            candleConfiguration.disableWhenLOS = false;
            candleConfiguration.stripType = LEDStripType.RGB;
            candleConfiguration.brightnessScalar = 1.0;
            candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
            candle.configAllSettings(candleConfiguration, 100);
        }

        setDefaultCommand(defaultCommand());
    }

    public static void setBrightness(double percent) {
        if (candle != null) {
            candle.configBrightnessScalar(percent, 100);
        }
    }

    public Command defaultCommand() {
        return run(() -> {
                    LEDSegment.BatteryIndicator.fullClear();
                    LEDSegment.DriverstationIndicator.fullClear();
                    LEDSegment.ExtraAIndicator.fullClear();
                    LEDSegment.ExtraBIndicator.fullClear();
                    LEDSegment.PivotEncoderIndicator.fullClear();
                    LightsControlModule.update();
                })
                .ignoringDisable(true);
    }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(
                () -> {
                    segment.clearAnimation();
                    segment.disableLEDs();
                });
    }

    // Condensed storage for animations, called when setting animations
    public static class LightsControlModule {
        enum mode {
            disabled,
            paused,
            manual,
            strobe,
            fade,
            fire,
            alignLeft,
            alignRight,
            alignCenter
        }

        static mode lightMode = mode.disabled;

        static double alignToleranceMin = 5;
        static double alignToleranceMax = 100;

        // #region

        public static boolean enabled = false;
        static BooleanSupplier hasPiece = () -> false;
        static BooleanSupplier isAligning = () -> false;
        static IntSupplier alignMode = () -> 0;

        public static void Supplier_hasPiece(BooleanSupplier sup) {
            hasPiece = sup;
        }

        public static void Supplier_isAligning(BooleanSupplier sup) {
            isAligning = sup;
        }

        public static void Supplier_alignMode(IntSupplier sup) {
            alignMode = sup;
        }

        // #endregion

        public static void update() {
            if (!enabled) {
                fade();
                return;
            }
            if (isAligning.getAsBoolean()) {
                int alignModeInt = alignMode.getAsInt();
                if (alignModeInt == ModeManager.ScoringMode.LeftCoral.ordinal()) {
                    alignLeft(10); // TODO: get distance to tag
                    return;
                }
                if (alignModeInt == ModeManager.ScoringMode.RightCoral.ordinal()) {
                    alignRight(10); // TODO: get distance to tag
                    return;
                }
                if (alignModeInt == ModeManager.ScoringMode.Algae.ordinal()) {
                    alignCenter(10); // TODO: get distance to tag
                    return;
                }
                fade();
                return;
            }
            if (hasPiece.getAsBoolean()) {
                strobe();
                return;
            }
            fire();
        }

        public static void clearAnimation() {
            if (lightMode == mode.paused) return;
            lightMode = mode.paused;

            LEDSegment.MainStrip.clearAnimation();
            LEDSegment.MainStripLeft.clearAnimation();
            LEDSegment.MainStripRight.clearAnimation();
        }

        public static void fullClear() {
            if (lightMode == mode.paused) return;
            lightMode = mode.paused;

            LEDSegment.MainStrip.fullClear();
            LEDSegment.MainStripLeft.fullClear();
            LEDSegment.MainStripRight.fullClear();
        }

        public static void strobe() {
            if (lightMode == mode.strobe) return;
            lightMode = mode.strobe;

            LEDSegment.MainStrip.setStrobeAnimation(white, 0.3);
            LEDSegment.MainStripLeft.clearAnimation();
            LEDSegment.MainStripRight.clearAnimation();
        }

        public static void fade() {
            if (lightMode == mode.fade) return;
            lightMode = mode.fade;

            LEDSegment.MainStrip.setFadeAnimation(orange, 0.5);
            LEDSegment.MainStripLeft.clearAnimation();
            LEDSegment.MainStripRight.clearAnimation();
        }

        public static void fire() {
            if (lightMode == mode.fire) return;
            lightMode = mode.fire;

            LEDSegment.MainStrip.clearAnimation();
            LEDSegment.MainStripLeft.setFireAnimation(0.2);
            LEDSegment.MainStripRight.setFireAnimation(0.2);
        }

        public static void alignLeft(double distance) {
            if (lightMode != mode.alignLeft) LEDSegment.MainStripRight.progressCount = 0;

            if (distance < alignToleranceMin) {
                LEDSegment.MainStrip.setColor(green);
                LEDSegment.MainStripLeft.clearAnimation();
                LEDSegment.MainStripRight.clearAnimation();
            } else if (distance < alignToleranceMax) {
                LEDSegment.MainStrip.clearAnimation();
                LEDSegment.MainStripLeft.setStrobeAnimation(blue, 0.25);
                if (lightMode != mode.alignLeft) LEDSegment.MainStripRight.setColor(red);
                updateProgressBar(LEDSegment.MainStripRight, distance);
            } else {
                LEDSegment.MainStrip.clearAnimation();
                LEDSegment.MainStripLeft.setStrobeAnimation(yellow, 0.15);
                if (lightMode != mode.alignLeft) LEDSegment.MainStripRight.setColor(red);
                updateProgressBar(LEDSegment.MainStripRight, distance);
            }

            lightMode = mode.alignLeft;
        }

        public static void alignRight(double distance) {
            if (lightMode != mode.alignRight) LEDSegment.MainStripLeft.progressCount = 0;

            if (distance < alignToleranceMin) {
                LEDSegment.MainStrip.setColor(green);
                LEDSegment.MainStripLeft.clearAnimation();
                LEDSegment.MainStripRight.clearAnimation();
            } else if (distance < alignToleranceMax) {
                LEDSegment.MainStrip.clearAnimation();
                if (lightMode != mode.alignRight) LEDSegment.MainStripLeft.setColor(red);
                LEDSegment.MainStripRight.setStrobeAnimation(blue, 0.3);
                updateProgressBar(LEDSegment.MainStripLeft, distance);
            } else {
                LEDSegment.MainStrip.clearAnimation();
                if (lightMode != mode.alignRight) LEDSegment.MainStripLeft.setColor(red);
                LEDSegment.MainStripRight.setStrobeAnimation(yellow, 0.15);
                updateProgressBar(LEDSegment.MainStripLeft, distance);
            }

            lightMode = mode.alignRight;
        }

        public static void alignCenter(double distance) {
            if (lightMode == mode.alignCenter) return;

            // The idea here: blink according to the directon we want to drive
            // Functionality is not correctly implemented at this time

            if (distance < alignToleranceMin) {
                LEDSegment.MainStrip.setColor(green);
                LEDSegment.MainStripLeft.clearAnimation();
                LEDSegment.MainStripRight.clearAnimation();
            } else if (distance < alignToleranceMax) {
                LEDSegment.MainStrip.clearAnimation();
                LEDSegment.MainStripLeft.setColor(purple); // Replace with a progress bar overlay
                LEDSegment.MainStripRight.setStrobeAnimation(blue, 0.25);
            } else {
                LEDSegment.MainStrip.clearAnimation();
                LEDSegment.MainStripLeft.setColor(purple);
                LEDSegment.MainStripRight.setStrobeAnimation(yellow, 0.15);
            }
            lightMode = mode.alignCenter;
        }

        static void updateProgressBar(LEDSegment segment, double distance) {
            int delta = findProgressDelta(segment, distance);
            if (delta == 0) return;
            if (!segment.reverseMode) {
                int currentLocation =
                        segment.startIndex
                                + segment.progressCount; // Current Location = the light after the
                // last active light
                if (delta > 0) {
                    // Turn on forwards
                    Color color = segment.progressOnColor;
                    candle.setLEDs(color.red, color.green, color.blue, 0, currentLocation, delta);
                } else {
                    // Turn off backwards
                    Color color = segment.progressOffColor;
                    candle.setLEDs(
                            color.red, color.green, color.blue, 0, currentLocation - 1, -delta);
                }
            } else {
                int currentLocation =
                        segment.startIndex
                                + segment.segmentSize
                                - 1
                                - segment.progressCount; // Current Location = the light after the
                // last active light
                if (delta > 0) {
                    // Turn on backwards
                    Color color = segment.progressOnColor;
                    candle.setLEDs(
                            color.red,
                            color.green,
                            color.blue,
                            0,
                            currentLocation - delta + 1,
                            delta);
                } else {
                    // Turn off forwards
                    Color color = segment.progressOffColor;
                    candle.setLEDs(
                            color.red, color.green, color.blue, 0, currentLocation + 1, -delta);
                }
            }
        }

        static int findProgressDelta(LEDSegment segment, double value) {
            value =
                    Math.max(alignToleranceMin, Math.min(alignToleranceMax, value))
                            - alignToleranceMin; // Clamp within the distance range
            int delta =
                    (int) Math.ceil(value / segment.progressValueDistance)
                            - segment.progressCount; // How many lights need to change
            return delta;
        }
    }

    public static enum LEDSegment {
        BatteryIndicator(0, 2, 0, false),
        DriverstationIndicator(2, 2, 1, false),
        ExtraAIndicator(4, 1, -1, false),
        ExtraBIndicator(5, 1, -1, false),
        PivotEncoderIndicator(6, 1, -1, false),
        AllianceIndicator(7, 1, -1, false),
        MainStrip(8, 108, 2, false),
        MainStripLeft(8, 53, 3, false, red, yellow),
        MainStripRight(61, 73, 4, true, red, yellow);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;
        public final boolean reverseMode;

        public int progressCount = 0;
        public final double progressValueDistance;
        public final Color progressOffColor;
        public final Color progressOnColor;

        private LEDSegment(
                int startIndex, int segmentSize, int animationSlot, boolean reverseMode) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
            this.reverseMode = reverseMode;
            this.progressValueDistance = 1 / segmentSize;
            this.progressOffColor = null;
            this.progressOnColor = null;
        }

        private LEDSegment(
                int startIndex,
                int segmentSize,
                int animationSlot,
                boolean reverseMode,
                Color progressOffColor,
                Color progressOnColor) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
            this.reverseMode = reverseMode;
            this.progressValueDistance = 1 / segmentSize;
            this.progressOffColor = progressOffColor;
            this.progressOnColor = progressOnColor;
        }

        public void setColor(Color color) {
            if (candle != null) {
                clearAnimation();
                candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
            }
        }

        private void setAnimation(Animation animation) {
            if (candle != null) {
                candle.animate(animation, animationSlot);
            }
        }

        public void fullClear() {
            if (candle != null) {
                clearAnimation();
                disableLEDs();
            }
        }

        public void clearAnimation() {
            if (candle != null) {
                candle.clearAnimation(animationSlot);
            }
        }

        public void disableLEDs() {
            if (candle != null) {
                setColor(black);
            }
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(
                    new ColorFlowAnimation(
                            color.red,
                            color.green,
                            color.blue,
                            0,
                            speed,
                            segmentSize,
                            (!reverseMode)
                                    ? Direction.Forward
                                    : Direction
                                            .Backward, // transmute reverseMode to Direction value
                            startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(
                            color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(
                    new LarsonAnimation(
                            color.red,
                            color.green,
                            color.blue,
                            0,
                            speed,
                            segmentSize,
                            (!reverseMode)
                                    ? BounceMode.Front
                                    : BounceMode.Back, // transmute reverseMode to BounceMode value
                            3,
                            startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(
                    new StrobeAnimation(
                            color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, reverseMode, startIndex));
        }

        public void setFireAnimation(double speed) {
            setAnimation(
                    new FireAnimation(1, speed, segmentSize, 0.5, 0.3, reverseMode, startIndex));
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        /**
         * Highly imperfect way of dimming the LEDs. It does not maintain color or accurately adjust
         * perceived brightness.
         *
         * @param dimFactor
         * @return The dimmed color
         */
        public Color dim(double dimFactor) {
            int newRed = (int) (ensureRange(red * dimFactor, 0, 200));
            int newGreen = (int) (ensureRange(green * dimFactor, 0, 200));
            int newBlue = (int) (ensureRange(blue * dimFactor, 0, 200));

            return new Color(newRed, newGreen, newBlue);
        }
    }

    private static double ensureRange(double value, double low, double upper) {
        return Math.max(low, Math.min(upper, value));
    }

    public static void disableLEDs() {
        setBrightness(0);
    }

    public static void enableLEDs() {
        setBrightness(.5);
    }
}
