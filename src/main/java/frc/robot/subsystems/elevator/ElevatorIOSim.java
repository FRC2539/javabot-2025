package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
    private double position;

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = position;
    }


}
