package frc.robot.subsystems.ModeManager;

import javax.swing.text.Position;
import java.util.Dictionary;
import java.util.Hashtable;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SuperstructureState {
    public static final Dictionary<Position,SuperstructureState> posDictionary = new Hashtable<>();

    public double elevatorheight;
    public double armheight;
    public double wristPosition;
    public Position position;
    public Position parent;

    public enum Position {
        None, 
        Home, L1, L1Prep, L2, L2Prep, L3, L3Prep, L4, L4Prep, L1Algae, L1AlgaePrep, L2Algae, L2AlgaePrep, L3Algae, L3AlgaePrep, L4Algae, L4AlgaePrep, Source, SourcePrep;

        public double value;

        private Position() {
            value = 0;
        }
    }

    public SuperstructureState(double elevatorheight, double armheight, double wristPosition, Position position, Position parent){
        this.armheight = armheight;
        this.elevatorheight = elevatorheight;
        this.position = position;
        this.parent = parent;
        this.wristPosition = wristPosition;

        posDictionary.put(position, this);
        
    }
}
