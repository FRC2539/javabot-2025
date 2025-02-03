package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsIO.Pattern;

public class LightsSubsystem extends SubsystemBase {
    private LightsIO lightsIO;

    public static Pattern red = new Pattern(0.61);
    public static Pattern orange = new Pattern(0.65);
    public static Pattern yellow = new Pattern(0.69);
    public static Pattern green = new Pattern(0.77);
    public static Pattern blue = new Pattern(0.87);
    public static Pattern purple = new Pattern(0.91);
    public static Pattern black = new Pattern(0.99);
    public static Pattern white = new Pattern(0.93);

    public static Pattern rainbow = new Pattern(-0.45);
    public static Pattern redFade = new Pattern(-0.17);

    public LightsSubsystem(LightsIO lightsIO) {
        this.lightsIO = lightsIO;
        setDefaultCommand(defaultCommand());
    }

    public Command defaultCommand() {
        return runOnce(() -> setPattern(orange));
    }

    public Command patternCommand(Pattern pattern) {
        return runOnce(() -> setPattern(pattern));
    }

    private void setPattern(Pattern pattern) {
        lightsIO.setPattern(pattern);
    }
}