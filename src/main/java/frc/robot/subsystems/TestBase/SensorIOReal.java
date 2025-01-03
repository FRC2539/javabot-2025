package frc.robot.subsystems.TestBase;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorIOReal implements SensorIO {
    private DigitalInput sensor = new DigitalInput(6);
    public boolean answer = false;
    public void updateInputs(SensorIOInputs inputs){
        boolean detected = sensor.get();
        answer = detected;
    }

    private boolean g(){
        return answer;
    }

    public BooleanSupplier getValue(){
        BooleanSupplier froot = this::g;
        return froot;
    }

}
