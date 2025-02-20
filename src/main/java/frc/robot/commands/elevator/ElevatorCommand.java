package frc.robot.commands.elevator;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double kG = 0.4;
    private double speed;
    


public ElevatorCommand(ElevatorSubsystem elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    
    addRequirements(elevator);
}

@Override
public void initialize() {}


@Override 
public void execute() {
  elevator.elevatorGo(speed);
    
}

@Override
    public boolean isFinished() {
       return false;
    }


}