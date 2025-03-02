package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;


public class ClimberCommand extends Command{
    private final ClimberSubsystem climber;
    private double speed;


public ClimberCommand(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
}

@Override
public void initialize() {
    
}

@Override 
public void execute() {
    climber.setClimber(speed);
}

@Override 
public void end(boolean interuppted) {
    climber.setClimber(0);
}

@Override 
public boolean isFinished() {
return false;
}
}
