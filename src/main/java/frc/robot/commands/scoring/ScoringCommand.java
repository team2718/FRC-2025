package frc.robot.commands.scoring;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.ScoringPositions;

public class ScoringCommand extends Command {
    private final SuperSystem supersystem;
    // private double position;

public ScoringCommand(SuperSystem supersystem) {
    this.supersystem = supersystem;
    // this.position = position;
    addRequirements(supersystem);
}

@Override 
public void initialize() {

}

@Override 
public void execute() {
    supersystem.setScoring();
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
