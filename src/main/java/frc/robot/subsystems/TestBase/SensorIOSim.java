package frc.robot.subsystems.TestBase;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SensorIOSim implements SensorIO {
    LoggedNetworkBoolean simSensor;
    public SensorIOSim(String name) {
        simSensor = new LoggedNetworkBoolean(name, false);
    }


    public boolean answer = false;

    public void updateInputs(SensorIOInputs inputs){
        boolean detected = simSensor.get();
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