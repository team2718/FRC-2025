package frc.robot.commands.endeffector;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorCommand extends Command{
    private final EndEffectorSubsystem endeffector;
    private double speed;
    
    public EndEffectorCommand(EndEffectorSubsystem endeffector, double speed) {
        this.endeffector = endeffector;
        this.speed = speed;
    addRequirements(endeffector);
   
}

@Override
public void initialize() {

}

@Override 
public void execute() {
    endeffector.setEndEffector(speed);
}

@Override
    public void end(boolean interuppted) {
        endeffector.setEndEffector(0);
     
    }
@Override
    public boolean isFinished() {
       return false;
    }
}
