package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.ScoringPositions;

public class SSElevatorCommand extends Command{
    private final SuperSystem supersystem;
    private ScoringPositions positions;

public SSElevatorCommand(SuperSystem supersystem, ScoringPositions positions) {
    this.supersystem = supersystem;
    this.positions = positions;
}
    
@Override
public void initialize() {

}

@Override
public void execute() {
    supersystem.setScoringPosition(this.positions);
    

}

@Override
    public void end(boolean interuppted) {
        // supersystem.setIntake();
     
    }

@Override
    public boolean isFinished() {
       return false;
    }
    
}
