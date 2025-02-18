package frc.robot.commands.elevator;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double kG = 0.4;


public ElevatorCommand(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    
    addRequirements(elevator);
}

@Override
public void initialize() {}


@Override 
public void execute() {

    
}


}