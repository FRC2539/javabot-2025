package frc.robot.subsystems.lights;

public interface LightsIO {

    public void setPattern(Pattern pattern);

    // The LightsIO class has no LightsIOInputs because there are no inputs

    public static class Pattern {
        public double PWMValue;

        public Pattern(double PWMValue) {
            this.PWMValue = PWMValue;
        }
    }
}