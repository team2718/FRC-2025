package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class SetClimberAtStart extends Command {
    private final ClimberSubsystem climber;

    public SetClimberAtStart(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setEngageMotor(1);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setEngageMotor(0);
    }

    @Override
    public boolean isFinished() {
        return climber.getEngagePosition() >= 1.5;
    }
}
