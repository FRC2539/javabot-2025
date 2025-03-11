package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LightsSubsystem extends SubsystemBase {
    public static final class LightsConstants {
        public static final int CANDLE_PORT = 13;
        public static final int CANDLE_LENGTH = 308;
        public static final Distance CANDLE_LED_SPACING =
                Units.Meters.of(1 / 120); // <==== GET THIS VALUE!!!

        public static final int SENSOR_PORT = 0;
    }

    private BooleanSupplier algaeMode = () -> false;

    private static final AddressableLED candle;
    private static final AddressableLEDBuffer candleBuffer;
    private static double brightnessPercent = 1;

    private static final AddressableLEDSim candleSim;

    private static final boolean isReal = true;

    static {
        if ((RobotBase.isReal() && isReal)) {
            // New Animation stuff
            candle = new AddressableLED(LightsConstants.CANDLE_PORT);
            candleSim = new AddressableLEDSim(candle);
            candleBuffer = new AddressableLEDBuffer(LightsConstants.CANDLE_LENGTH);
            candle.setLength(candleBuffer.getLength());

            candle.setData(candleBuffer);
            candle.start();
        } else {
            candle = null;
            candleBuffer = null;
            candleSim = null;
        }
    }

    // Team m_Colors
    public static final m_Color orange = new m_Color(200, 25, 100);
    public static final m_Color black = new m_Color(0, 0, 0);

    // Game piece m_Colors
    public static final m_Color yellow = new m_Color(242, 60, 0);
    public static final m_Color purple = new m_Color(184, 0, 185);

    // Indicator m_Colors
    public static final m_Color white = new m_Color(255, 230, 220);
    public static final m_Color green = new m_Color(56, 209, 0);
    public static final m_Color blue = new m_Color(8, 32, 255);
    public static final m_Color red = new m_Color(255, 0, 0);

    public LightsSubsystem() {
        if (candle != null) {
            /*
            CANdleConfiguration candleConfiguration = new CANdleConfiguration();
            candleConfiguration.statusLedOffWhenActive = true;
            candleConfiguration.disableWhenLOS = false;
            candleConfiguration.stripType = LEDStripType.RGB;
            candleConfiguration.brightnessScalar = 1.0;
            candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
            candle.configAllSettings(candleConfiguration, 100);
            */
        }

        setDefaultCommand(defaultCommand());
    }

    public static void setBrightness(double percent) {
        if (candle != null) {
            // candle.configBrightnessScalar(percent, 100);
            brightnessPercent = percent;
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

                        LEDSegment.MainStrip.setFadeAnimation(orange, 0.2);
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
        public final AddressableLEDBufferView m_view;
        public Command animationer = Commands.none();

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
            if (candleBuffer != null) m_view = candleBuffer.createView(startIndex, animationSlot);
            else m_view = null;
        }

        public void setColor(m_Color m_Color) {
            if (candle != null) {
                clearAnimation();
                LEDPattern pattern =
                        LEDPattern.solid(m_Color.convert())
                                .atBrightness(Units.Percent.of(brightnessPercent));
                pattern.applyTo(m_view);
                candle.setData(candleBuffer);

                // candle.setLEDs(m_Color.red, m_Color.green, m_Color.blue, 0, startIndex,
                // segmentSize);
            }
        }

        private void setAnimation(LEDPattern animation) {
            if (candle != null) {
                animation.atBrightness(Units.Percent.of(brightnessPercent)).applyTo(m_view);
                candle.setData(candleBuffer);
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
                CANdle t = new CANdle(animationSlot);
                t.clearAnimation(animationSlot);
            }
        }

        public void disableLEDs() {
            if (candle != null) {
                setColor(black);
            }
        }

        public void setFlowAnimation(m_Color m_Color, double speed) {
            setAnimation(
                    LEDPattern.gradient(
                                    GradientType.kContinuous, m_Color.convert(), black.convert())
                            .scrollAtAbsoluteSpeed(
                                    Units.MetersPerSecond.of(speed),
                                    LightsConstants.CANDLE_LED_SPACING));

            /*
            setAnimation(
                    new m_ColorFlowAnimation(
                            m_Color.red,
                            m_Color.green,
                            m_Color.blue,
                            0,
                            speed,
                            segmentSize,
                            Direction.Forward,
                            startIndex));
                            */
        }

        public void setFadeAnimation(m_Color m_Color, double speed) {
            setAnimation(LEDPattern.solid(m_Color.convert()).breathe(Units.Seconds.of(speed)));
            // setAnimation( new SingleFadeAnimation(m_Color.red, m_Color.green, m_Color.blue, 0,
            // speed, segmentSize, startIndex));
        }

        public void setBandAnimation(
                m_Color m_Color,
                double speed) { // NEEDS A COMMAND STRUCTURE TO BOUNCE; CURRENTLY NOT WORKING AS
            // INTENDED
            setAnimation(
                    LEDPattern.solid(m_Color.convert())
                            .scrollAtAbsoluteSpeed(
                                    Units.MetersPerSecond.of(speed),
                                    LightsConstants.CANDLE_LED_SPACING));

            /*
            setAnimation(
                    new LarsonAnimation(
                            m_Color.red,
                            m_Color.green,
                            m_Color.blue,
                            0,
                            speed,
                            segmentSize,
                            BounceMode.Front,
                            3,
                            startIndex));
                            */
        }

        public void setStrobeAnimation(m_Color m_Color, double speed) {
            setAnimation(LEDPattern.solid(m_Color.convert()).blink(Units.Seconds.of(speed)));
            // setAnimation(
            //        new StrobeAnimation(
            //                m_Color.red, m_Color.green, m_Color.blue, 0, speed, segmentSize,
            // startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(
                    LEDPattern.rainbow(255, 128)
                            .scrollAtAbsoluteSpeed(
                                    Units.MetersPerSecond.of(speed),
                                    LightsConstants.CANDLE_LED_SPACING));
            // setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
    }

    public static class m_Color {
        public int red;
        public int green;
        public int blue;

        public m_Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        /**
         * Highly imperfect way of dimming the LEDs. It does not maintain m_Color or accurately
         * adjust perceived brightness.
         *
         * @param dimFactor
         * @return The dimmed Color
         */
        public m_Color dim(double dimFactor) {
            int newRed = (int) (ensureRange(red * dimFactor, 0, 200));
            int newGreen = (int) (ensureRange(green * dimFactor, 0, 200));
            int newBlue = (int) (ensureRange(blue * dimFactor, 0, 200));

            return new m_Color(newRed, newGreen, newBlue);
        }

        public Color convert() {
            return new Color(red, green, blue);
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
