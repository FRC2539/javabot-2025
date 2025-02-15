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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LightsSubsystem extends SubsystemBase {
    public static final class LightsConstants {
        public static final int CANDLE_PORT = 12;

        public static final int SENSOR_PORT = 0;
    }

    private BooleanSupplier hasPiece = () -> false;

    private static final CANdle candle;

    static {
        if (false && RobotBase.isReal()) {
            candle = new CANdle(LightsConstants.CANDLE_PORT);
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
        return run(
                () -> {
                    LEDSegment.BatteryIndicator.fullClear();
                    LEDSegment.DriverstationIndicator.fullClear();
                    LEDSegment.ExtraAIndicator.fullClear();
                    LEDSegment.ExtraBIndicator.fullClear();
                    LEDSegment.PivotEncoderIndicator.fullClear();

                    if (hasPiece.getAsBoolean()) {
                        LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(
                                LightsSubsystem.white, 0.3);
                    } else {
                        LEDSegment.MainStrip.setColor(orange);
                    }
                });
    }

    public void setHasPieceSupplier(BooleanSupplier hasPiece) {
        this.hasPiece = hasPiece;
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
        setBrightness(1);
    }
}
