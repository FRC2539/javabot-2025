package frc.robot.subsystems.ModeManager;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperstructureStateManager {
    public static SuperstructureState currentState = new SuperstructureState(0,0,0,SuperstructureState.Position.None, SuperstructureState.Position.None);

    public SuperstructureState.Position targetPostition = SuperstructureState.Position.None;

    public List<SuperstructureState.Position> outList = new ArrayList<>();

    public List<SuperstructureState.Position> inList = new ArrayList<>();

    public void update() {
        currentState.armheight = getArmHeight();
        currentState.elevatorheight = getElevatorHeight();
    }
    public double getArmHeight() { return 0; }
    public double getElevatorHeight() { return 0; }


    public void setFinalTarget(SuperstructureState.Position targetPosition) {
        outList.clear();
        // Set the [ (A') => (C') ] target of the system (initialize pathing command)
        boolean found = false;

        for (int i = 0; i < 20; i++) {
            SuperstructureState head = SuperstructureState.posDictionary.get(targetPosition);
            outList.add(head.position);
            if (head.position == SuperstructureState.Position.None) {
                found = true;
                break;
            }
            targetPosition = head.parent;
        }

        if (!found) {
            throw new RuntimeException("AY CARAMBA. (Your state machine didnt resolve.)");
        }
    }
    
    public void setCurrentTarget(SuperstructureState.Position targetPosition) {
        
        // Set the [ (A) => (A') ] target of the system (initialize movement command)
    }

    public Command inList(){
        for()
        
    }

    public Command goToPosition(SuperstructureState.Position targetPosition) {
        Command actualMovingCommand = Commands.none();
        Command inListAdder = Commands.runOnce(inList.add())

        return actualMovingCommand;
    }

    public Command MoveSystem() {
        return null;
    }
    
}
