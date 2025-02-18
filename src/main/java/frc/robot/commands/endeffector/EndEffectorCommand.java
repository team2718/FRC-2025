package frc.robot.commands.endeffector;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorCommand extends Command{
    private final EndEffectorSubsystem endeffector;

public EndEffectorCommand(EndEffectorSubsystem endeffector, double speed) {
    this.endeffector = endeffector;
    addRequirements(endeffector);
}

@Override
public void initialize() {

}

@Override 
public void execute() {
    endeffector.setEndEffector(0.8);
}

@Override
    public void end(boolean interuppted) {
        endeffector.setEndEffector(0);
     
    }
@Override
    public boolean isFinished() {
       return true;
    }
}
