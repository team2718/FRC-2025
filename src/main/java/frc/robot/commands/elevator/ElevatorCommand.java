package frc.robot.commands.elevator;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private double position;

    public ElevatorCommand(ElevatorSubsystem elevator, double position) {
        this.elevator = elevator;
        this.position = position;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.setTargetPosition(position);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}