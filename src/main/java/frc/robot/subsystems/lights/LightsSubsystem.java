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

    String currentAnimation = "";

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

<<<<<<< HEAD
    // region importAnimation
    CustomAnimation importAnimation =
    new CustomAnimation(candle,8,14, new AnimDelta( new AnimColorSegment[] { new AnimColorSegment(new Color(255,20,0),8,1),
        new AnimColorSegment(new Color(255,20,0),9,1),
        new AnimColorSegment(black,10,9),
        new AnimColorSegment(new Color(255,20,0),19,1),
        new AnimColorSegment(new Color(255,20,0),20,1),
        new AnimColorSegment(new Color(255,20,0),21,1),
        new AnimColorSegment(new Color(255,20,0),22,1),
        new AnimColorSegment(new Color(255,20,0),23,1),
        new AnimColorSegment(black,24,9),
        new AnimColorSegment(new Color(255,20,0),33,1),
        new AnimColorSegment(new Color(255,20,0),34,1),
        new AnimColorSegment(new Color(255,20,0),35,1),
        new AnimColorSegment(new Color(255,20,0),36,1),
        new AnimColorSegment(new Color(255,20,0),37,1),
        new AnimColorSegment(black,38,9),
        new AnimColorSegment(new Color(255,20,0),47,1),
        new AnimColorSegment(new Color(255,20,0),48,1),
        new AnimColorSegment(new Color(255,20,0),49,1),
        new AnimColorSegment(new Color(255,20,0),50,1),
        new AnimColorSegment(new Color(255,20,0),51,1),
        new AnimColorSegment(black,52,9),
        new AnimColorSegment(new Color(255,20,0),61,1),
        new AnimColorSegment(new Color(255,20,0),62,1),
        new AnimColorSegment(new Color(255,20,0),63,1),
        new AnimColorSegment(new Color(255,20,0),64,1),
        new AnimColorSegment(new Color(255,20,0),65,1),
        new AnimColorSegment(black,66,10),
        new AnimColorSegment(new Color(255,20,0),76,1),
        new AnimColorSegment(new Color(255,20,0),77,1),
        new AnimColorSegment(new Color(255,20,0),78,1),
        new AnimColorSegment(new Color(255,20,0),79,1),
        new AnimColorSegment(new Color(255,20,0),80,1),
        new AnimColorSegment(black,81,9),
        new AnimColorSegment(new Color(255,20,0),90,1),
        new AnimColorSegment(new Color(255,20,0),91,1),
        new AnimColorSegment(new Color(255,20,0),92,1),
        new AnimColorSegment(new Color(255,20,0),93,1),
        new AnimColorSegment(new Color(255,20,0),94,1),
        new AnimColorSegment(black,95,9),
        new AnimColorSegment(new Color(255,20,0),104,1),
        new AnimColorSegment(new Color(255,20,0),105,1),
        new AnimColorSegment(new Color(255,20,0),106,1),
        new AnimColorSegment(new Color(255,20,0),107,1),
        new AnimColorSegment(new Color(255,20,0),108,1),
        new AnimColorSegment(black,109,9),
        new AnimColorSegment(new Color(255,20,0),118,1),
        new AnimColorSegment(new Color(255,20,0),119,1),
        new AnimColorSegment(new Color(255,20,0),120,1),
        new AnimColorSegment(new Color(255,20,0),121,1),
        new AnimColorSegment(new Color(255,20,0),122,1),
        new AnimColorSegment(black,123,9),
        new AnimColorSegment(new Color(255,20,0),132,1),
        new AnimColorSegment(new Color(255,20,0),133,1),
        }), new Hashtable<Integer, AnimDelta>() {{
        put(1, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(new Color(255,20,0),11,1),
        new AnimColorSegment(black,20,1),
        new AnimColorSegment(new Color(255,20,0),25,1),
        new AnimColorSegment(black,34,1),
        new AnimColorSegment(new Color(255,20,0),39,1),
        new AnimColorSegment(black,48,1),
        new AnimColorSegment(new Color(255,20,0),53,1),
        new AnimColorSegment(black,62,1),
        new AnimColorSegment(new Color(255,20,0),67,1),
        new AnimColorSegment(new Color(255,20,0),74,1),
        new AnimColorSegment(black,79,1),
        new AnimColorSegment(new Color(255,20,0),88,1),
        new AnimColorSegment(black,93,1),
        new AnimColorSegment(new Color(255,20,0),102,1),
        new AnimColorSegment(black,107,1),
        new AnimColorSegment(new Color(255,20,0),116,1),
        new AnimColorSegment(black,121,1),
        new AnimColorSegment(new Color(255,20,0),130,1),
        }));
        put(2, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(new Color(255,20,0),12,1),
        new AnimColorSegment(black,21,1),
        new AnimColorSegment(new Color(255,20,0),26,1),
        new AnimColorSegment(black,35,1),
        new AnimColorSegment(new Color(255,20,0),40,1),
        new AnimColorSegment(black,49,1),
        new AnimColorSegment(new Color(255,20,0),54,1),
        new AnimColorSegment(black,63,1),
        new AnimColorSegment(new Color(255,20,0),68,1),
        new AnimColorSegment(new Color(255,20,0),73,1),
        new AnimColorSegment(black,78,1),
        new AnimColorSegment(new Color(255,20,0),87,1),
        new AnimColorSegment(black,92,1),
        new AnimColorSegment(new Color(255,20,0),101,1),
        new AnimColorSegment(black,106,1),
        new AnimColorSegment(new Color(255,20,0),115,1),
        new AnimColorSegment(black,120,1),
        new AnimColorSegment(new Color(255,20,0),129,1),
        }));
        put(3, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,8,1),
        new AnimColorSegment(new Color(255,20,0),13,1),
        new AnimColorSegment(black,22,1),
        new AnimColorSegment(new Color(255,20,0),27,1),
        new AnimColorSegment(black,36,1),
        new AnimColorSegment(new Color(255,20,0),41,1),
        new AnimColorSegment(black,50,1),
        new AnimColorSegment(new Color(255,20,0),55,1),
        new AnimColorSegment(black,64,1),
        new AnimColorSegment(new Color(255,20,0),69,1),
        new AnimColorSegment(new Color(255,20,0),72,1),
        new AnimColorSegment(black,77,1),
        new AnimColorSegment(new Color(255,20,0),86,1),
        new AnimColorSegment(black,91,1),
        new AnimColorSegment(new Color(255,20,0),100,1),
        new AnimColorSegment(black,105,1),
        new AnimColorSegment(new Color(255,20,0),114,1),
        new AnimColorSegment(black,119,1),
        new AnimColorSegment(new Color(255,20,0),128,1),
        new AnimColorSegment(black,133,1),
        }));
        put(4, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,9,1),
        new AnimColorSegment(new Color(255,20,0),14,1),
        new AnimColorSegment(black,23,1),
        new AnimColorSegment(new Color(255,20,0),28,1),
        new AnimColorSegment(black,37,1),
        new AnimColorSegment(new Color(255,20,0),42,1),
        new AnimColorSegment(black,51,1),
        new AnimColorSegment(new Color(255,20,0),56,1),
        new AnimColorSegment(black,65,1),
        new AnimColorSegment(new Color(255,20,0),70,1),
        new AnimColorSegment(new Color(255,20,0),71,1),
        new AnimColorSegment(black,76,1),
        new AnimColorSegment(new Color(255,20,0),85,1),
        new AnimColorSegment(black,90,1),
        new AnimColorSegment(new Color(255,20,0),99,1),
        new AnimColorSegment(black,104,1),
        new AnimColorSegment(new Color(255,20,0),113,1),
        new AnimColorSegment(black,118,1),
        new AnimColorSegment(new Color(255,20,0),127,1),
        new AnimColorSegment(black,132,1),
        }));
        put(5, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,10,1),
        new AnimColorSegment(new Color(255,20,0),15,1),
        new AnimColorSegment(black,24,1),
        new AnimColorSegment(new Color(255,20,0),29,1),
        new AnimColorSegment(black,38,1),
        new AnimColorSegment(new Color(255,20,0),43,1),
        new AnimColorSegment(black,52,1),
        new AnimColorSegment(new Color(255,20,0),57,1),
        new AnimColorSegment(black,66,1),
        new AnimColorSegment(black,75,1),
        new AnimColorSegment(new Color(255,20,0),84,1),
        new AnimColorSegment(black,89,1),
        new AnimColorSegment(new Color(255,20,0),98,1),
        new AnimColorSegment(black,103,1),
        new AnimColorSegment(new Color(255,20,0),112,1),
        new AnimColorSegment(black,117,1),
        new AnimColorSegment(new Color(255,20,0),126,1),
        new AnimColorSegment(black,131,1),
        }));
        put(6, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,11,1),
        new AnimColorSegment(new Color(255,20,0),16,1),
        new AnimColorSegment(black,25,1),
        new AnimColorSegment(new Color(255,20,0),30,1),
        new AnimColorSegment(black,39,1),
        new AnimColorSegment(new Color(255,20,0),44,1),
        new AnimColorSegment(black,53,1),
        new AnimColorSegment(new Color(255,20,0),58,1),
        new AnimColorSegment(black,67,1),
        new AnimColorSegment(black,74,1),
        new AnimColorSegment(new Color(255,20,0),83,1),
        new AnimColorSegment(black,88,1),
        new AnimColorSegment(new Color(255,20,0),97,1),
        new AnimColorSegment(black,102,1),
        new AnimColorSegment(new Color(255,20,0),111,1),
        new AnimColorSegment(black,116,1),
        new AnimColorSegment(new Color(255,20,0),125,1),
        new AnimColorSegment(black,130,1),
        }));
        put(7, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,12,1),
        new AnimColorSegment(new Color(255,20,0),17,1),
        new AnimColorSegment(black,26,1),
        new AnimColorSegment(new Color(255,20,0),31,1),
        new AnimColorSegment(black,40,1),
        new AnimColorSegment(new Color(255,20,0),45,1),
        new AnimColorSegment(black,54,1),
        new AnimColorSegment(new Color(255,20,0),59,1),
        new AnimColorSegment(black,68,1),
        new AnimColorSegment(black,73,1),
        new AnimColorSegment(new Color(255,20,0),82,1),
        new AnimColorSegment(black,87,1),
        new AnimColorSegment(new Color(255,20,0),96,1),
        new AnimColorSegment(black,101,1),
        new AnimColorSegment(new Color(255,20,0),110,1),
        new AnimColorSegment(black,115,1),
        new AnimColorSegment(new Color(255,20,0),124,1),
        new AnimColorSegment(black,129,1),
        }));
        put(8, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,13,1),
        new AnimColorSegment(new Color(255,20,0),18,1),
        new AnimColorSegment(black,27,1),
        new AnimColorSegment(new Color(255,20,0),32,1),
        new AnimColorSegment(black,41,1),
        new AnimColorSegment(new Color(255,20,0),46,1),
        new AnimColorSegment(black,55,1),
        new AnimColorSegment(new Color(255,20,0),60,1),
        new AnimColorSegment(black,69,1),
        new AnimColorSegment(black,72,1),
        new AnimColorSegment(new Color(255,20,0),81,1),
        new AnimColorSegment(black,86,1),
        new AnimColorSegment(new Color(255,20,0),95,1),
        new AnimColorSegment(black,100,1),
        new AnimColorSegment(new Color(255,20,0),109,1),
        new AnimColorSegment(black,114,1),
        new AnimColorSegment(new Color(255,20,0),123,1),
        new AnimColorSegment(black,128,1),
        }));
        put(9, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,14,1),
        new AnimColorSegment(new Color(255,20,0),19,1),
        new AnimColorSegment(black,28,1),
        new AnimColorSegment(new Color(255,20,0),33,1),
        new AnimColorSegment(black,42,1),
        new AnimColorSegment(new Color(255,20,0),47,1),
        new AnimColorSegment(black,56,1),
        new AnimColorSegment(new Color(255,20,0),61,1),
        new AnimColorSegment(black,70,2),
        new AnimColorSegment(new Color(255,20,0),80,1),
        new AnimColorSegment(black,85,1),
        new AnimColorSegment(new Color(255,20,0),94,1),
        new AnimColorSegment(black,99,1),
        new AnimColorSegment(new Color(255,20,0),108,1),
        new AnimColorSegment(black,113,1),
        new AnimColorSegment(new Color(255,20,0),122,1),
        new AnimColorSegment(black,127,1),
        }));
        put(10, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,15,1),
        new AnimColorSegment(new Color(255,20,0),20,1),
        new AnimColorSegment(black,29,1),
        new AnimColorSegment(new Color(255,20,0),34,1),
        new AnimColorSegment(black,43,1),
        new AnimColorSegment(new Color(255,20,0),48,1),
        new AnimColorSegment(black,57,1),
        new AnimColorSegment(new Color(255,20,0),62,1),
        new AnimColorSegment(new Color(255,20,0),79,1),
        new AnimColorSegment(black,84,1),
        new AnimColorSegment(new Color(255,20,0),93,1),
        new AnimColorSegment(black,98,1),
        new AnimColorSegment(new Color(255,20,0),107,1),
        new AnimColorSegment(black,112,1),
        new AnimColorSegment(new Color(255,20,0),121,1),
        new AnimColorSegment(black,126,1),
        }));
        put(11, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(black,16,1),
        new AnimColorSegment(new Color(255,20,0),21,1),
        new AnimColorSegment(black,30,1),
        new AnimColorSegment(new Color(255,20,0),35,1),
        new AnimColorSegment(black,44,1),
        new AnimColorSegment(new Color(255,20,0),49,1),
        new AnimColorSegment(black,58,1),
        new AnimColorSegment(new Color(255,20,0),63,1),
        new AnimColorSegment(new Color(255,20,0),78,1),
        new AnimColorSegment(black,83,1),
        new AnimColorSegment(new Color(255,20,0),92,1),
        new AnimColorSegment(black,97,1),
        new AnimColorSegment(new Color(255,20,0),106,1),
        new AnimColorSegment(black,111,1),
        new AnimColorSegment(new Color(255,20,0),120,1),
        new AnimColorSegment(black,125,1),
        }));
        put(12, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(new Color(255,20,0),8,1),
        new AnimColorSegment(black,17,1),
        new AnimColorSegment(new Color(255,20,0),22,1),
        new AnimColorSegment(black,31,1),
        new AnimColorSegment(new Color(255,20,0),36,1),
        new AnimColorSegment(black,45,1),
        new AnimColorSegment(new Color(255,20,0),50,1),
        new AnimColorSegment(black,59,1),
        new AnimColorSegment(new Color(255,20,0),64,1),
        new AnimColorSegment(new Color(255,20,0),77,1),
        new AnimColorSegment(black,82,1),
        new AnimColorSegment(new Color(255,20,0),91,1),
        new AnimColorSegment(black,96,1),
        new AnimColorSegment(new Color(255,20,0),105,1),
        new AnimColorSegment(black,110,1),
        new AnimColorSegment(new Color(255,20,0),119,1),
        new AnimColorSegment(black,124,1),
        new AnimColorSegment(new Color(255,20,0),133,1),
        }));
        put(13, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(new Color(255,20,0),9,1),
        new AnimColorSegment(black,18,1),
        new AnimColorSegment(new Color(255,20,0),23,1),
        new AnimColorSegment(black,32,1),
        new AnimColorSegment(new Color(255,20,0),37,1),
        new AnimColorSegment(black,46,1),
        new AnimColorSegment(new Color(255,20,0),51,1),
        new AnimColorSegment(black,60,1),
        new AnimColorSegment(new Color(255,20,0),65,1),
        new AnimColorSegment(new Color(255,20,0),76,1),
        new AnimColorSegment(black,81,1),
        new AnimColorSegment(new Color(255,20,0),90,1),
        new AnimColorSegment(black,95,1),
        new AnimColorSegment(new Color(255,20,0),104,1),
        new AnimColorSegment(black,109,1),
        new AnimColorSegment(new Color(255,20,0),118,1),
        new AnimColorSegment(black,123,1),
        new AnimColorSegment(new Color(255,20,0),132,1),
        }));
        put(14, new AnimDelta(new AnimColorSegment[] {new AnimColorSegment(new Color(255,20,0),10,1),
        new AnimColorSegment(black,19,1),
        new AnimColorSegment(new Color(255,20,0),24,1),
        new AnimColorSegment(black,33,1),
        new AnimColorSegment(new Color(255,20,0),38,1),
        new AnimColorSegment(black,47,1),
        new AnimColorSegment(new Color(255,20,0),52,1),
        new AnimColorSegment(black,61,1),
        new AnimColorSegment(new Color(255,20,0),66,1),
        new AnimColorSegment(new Color(255,20,0),75,1),
        new AnimColorSegment(black,80,1),
        new AnimColorSegment(new Color(255,20,0),89,1),
        new AnimColorSegment(black,94,1),
        new AnimColorSegment(new Color(255,20,0),103,1),
        new AnimColorSegment(black,108,1),
        new AnimColorSegment(new Color(255,20,0),117,1),
        new AnimColorSegment(black,122,1),
        new AnimColorSegment(new Color(255,20,0),131,1),
        }));}},
        "importAnimation");
    // endregion

    CustomAnimation testAnimation =
            new CustomAnimation(
                    candle,
                    8,
=======
    CustomAnimation testAnimation =
            new CustomAnimation(
                    candle,
                    0,
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
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
<<<<<<< HEAD
                    },
                    "testAnimation");
=======
                    });
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)

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
                            LEDSegment.MainStrip.setCustomAnimation(importAnimation, 1);
                            // LEDSegment.MainStrip.setColor(orange);
                        }
                    } else {
<<<<<<< HEAD
                        LEDSegment.MainStrip.setCustomAnimation(importAnimation, 1);
=======
                        LEDSegment.MainStrip.setCustomAnimation(testAnimation, 1);
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
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
        MainStrip(8, 127, 2);

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
<<<<<<< HEAD
        final String id;
=======
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)

        CustomAnimation(
                CANdle candle,
                int pixelOffset,
                int frameCount,
                AnimDelta frameDeltaInitial,
<<<<<<< HEAD
                Dictionary<Integer, AnimDelta> frameDeltas,
                String id) {
=======
                Dictionary<Integer, AnimDelta> frameDeltas) {
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
            this.candle = candle;
            this.pixelOffset = pixelOffset;
            this.frameCount = frameCount;
            this.frameDeltaInitial = frameDeltaInitial;
            this.frameDeltas = frameDeltas;
<<<<<<< HEAD
            this.id = id;
=======
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
        }

        CustomAnimation(
                CANdle candle,
                int pixelOffset,
                int frameCount,
                AnimDelta frameDeltaInitial,
                Dictionary<Integer, AnimDelta> frameDeltas,
<<<<<<< HEAD
                double fps,
                String id) {
=======
                double fps) {
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
            this.candle = candle;
            this.pixelOffset = pixelOffset;
            this.frameCount = frameCount;
            this.frameDeltaInitial = frameDeltaInitial;
            this.frameDeltas = frameDeltas;
            this.fps = fps;
<<<<<<< HEAD
            this.id = id;
=======
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
        }

        public void init() {
            frameCurrent = 0;
            frameDeltaInitial.apply(candle);
        }

        public void update() {
<<<<<<< HEAD
            if (currentAnimation != id) {
                init();
                currentAnimation = id;
                return;
            }
            frameCurrent++;
            AnimDelta delta = frameDeltas.get(frameCurrent);
            if (delta != null) delta.apply(candle);
            if (frameCurrent >= frameCount) {
                frameCurrent = 0;
            }
=======
            frameCurrent++;
            if (frameCurrent >= frameCount) {
                init();
                return;
            }
            AnimDelta delta = frameDeltas.get(frameCurrent);
            if (delta != null) delta.apply(candle);
>>>>>>> e2dcb8f (Added Logistics, Pending Testing)
        }
    }

    // #endregion
}
