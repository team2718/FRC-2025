package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.ScoringPositions;

public class SSElevatorCommand extends Command{
    private final SuperSystem supersystem;
    private double position;

public SSElevatorCommand(SuperSystem supersystem) {
    this.supersystem = supersystem;
   

    addRequirements(supersystem);
    }
    
@Override
public void initialize() {

}

@Override
public void execute() {
    supersystem.setScoringPosition(ScoringPositions.L1);
    

}

@Override
    public void end(boolean interuppted) {
        supersystem.setIntake();
     
    }

@Override
    public boolean isFinished() {
       return false;
    }
    
}
