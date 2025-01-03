package frc.robot.subsystems.TestBase;

import java.util.function.BooleanSupplier;

public interface SensorIO {
    public void updateInputs(SensorIOInputs inputs);
   

    public class SensorIOInputs {
        private boolean detected;
        
        
    }

    public BooleanSupplier getValue();

    

}
