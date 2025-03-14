package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.BooleanSupplier;

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

    CustomAnimation testAnimation =
            new CustomAnimation(
                    candle,
                    0,
                    30,
                    new AnimDelta(
                            new AnimColorSegment[] {
                                new AnimColorSegment(orange, 8, 25),
                                new AnimColorSegment(blue, 25 + 8, 25),
                                new AnimColorSegment(red, 50 + 8, 25)
                            }),
                    new Hashtable<Integer, AnimDelta>() {
                        {
                            put(
                                    10,
                                    new AnimDelta(
                                            new AnimColorSegment[] {
                                                new AnimColorSegment(blue, 8, 25),
                                                new AnimColorSegment(red, 25 + 8, 25),
                                                new AnimColorSegment(orange, 50 + 8, 25)
                                            }));
                            put(
                                    20,
                                    new AnimDelta(
                                            new AnimColorSegment[] {
                                                new AnimColorSegment(red, 8, 25),
                                                new AnimColorSegment(orange, 25 + 8, 25),
                                                new AnimColorSegment(blue, 50 + 8, 25)
                                            }));
                        }
                    });

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

                    if (DriverStation.isEnabled()) {
                        if (algaeMode.getAsBoolean()) {
                            LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(
                                    LightsSubsystem.white, 0.3);
                        } else {
                            LEDSegment.MainStrip.setColor(orange);
                        }
                    } else {
                        LEDSegment.MainStrip.setCustomAnimation(testAnimation, 1);
                        // LEDSegment.MainStrip.setFadeAnimation(orange, 0.2);
                    }
                })
                .ignoringDisable(true);
    }

    public void setAlgaeModeSupplier(BooleanSupplier algaeMode) {
        this.algaeMode = algaeMode;
    }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(
                () -> {
                    segment.clearAnimation();
                    segment.disableLEDs();
                });
    }

    public static enum LEDSegment {
        BatteryIndicator(0, 2, 0),
        DriverstationIndicator(2, 2, 1),
        ExtraAIndicator(4, 1, -1),
        ExtraBIndicator(5, 1, -1),
        PivotEncoderIndicator(6, 1, -1),
        AllianceIndicator(7, 1, -1),
        MainStrip(8, 300, 2);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
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
                            Direction.Forward,
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
                            BounceMode.Front,
                            3,
                            startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(
                    new StrobeAnimation(
                            color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }

        public void setCustomAnimation(CustomAnimation anim, double speed) {
            candle.clearAnimation(animationSlot);
            anim.update();
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

    // #region Custom Animations

    class AnimColorSegment {
        public final Color color;
        public final int start;
        public final int length;

        AnimColorSegment(Color color, int start, int length) {
            this.color = color;
            this.start = start;
            this.length = length;
        }
    }

    class AnimDelta {
        public final AnimColorSegment[] deltas;

        AnimDelta(AnimColorSegment[] deltas) {
            this.deltas = deltas;
        }

        public void apply(CANdle candle) {
            for (AnimColorSegment cur : deltas) {
                Color color = cur.color;
                candle.setLEDs(color.red, color.green, color.blue, 0, cur.start, cur.length);
            }
        }
    }

    class CustomAnimation {
        final CANdle candle;
        final int pixelOffset;
        final int frameCount;
        public int frameCurrent = 0;
        public double fps = 5;
        final AnimDelta frameDeltaInitial;
        final Dictionary<Integer, AnimDelta> frameDeltas;

        CustomAnimation(
                CANdle candle,
                int pixelOffset,
                int frameCount,
                AnimDelta frameDeltaInitial,
                Dictionary<Integer, AnimDelta> frameDeltas) {
            this.candle = candle;
            this.pixelOffset = pixelOffset;
            this.frameCount = frameCount;
            this.frameDeltaInitial = frameDeltaInitial;
            this.frameDeltas = frameDeltas;
        }

        CustomAnimation(
                CANdle candle,
                int pixelOffset,
                int frameCount,
                AnimDelta frameDeltaInitial,
                Dictionary<Integer, AnimDelta> frameDeltas,
                double fps) {
            this.candle = candle;
            this.pixelOffset = pixelOffset;
            this.frameCount = frameCount;
            this.frameDeltaInitial = frameDeltaInitial;
            this.frameDeltas = frameDeltas;
            this.fps = fps;
        }

        public void init() {
            frameCurrent = 0;
            frameDeltaInitial.apply(candle);
        }

        public void update() {
            frameCurrent++;
            if (frameCurrent >= frameCount) {
                init();
                return;
            }
            AnimDelta delta = frameDeltas.get(frameCurrent);
            if (delta != null) delta.apply(candle);
        }
    }

    // #endregion
}
