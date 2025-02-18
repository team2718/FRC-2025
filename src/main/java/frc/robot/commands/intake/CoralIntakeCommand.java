package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {
    private final CoralIntakeSubsystem intake;
    public CoralIntakeCommand(CoralIntakeSubsystem intake, double speed) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setStateIntake();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interuppted) {
        intake.setStateHold();
    }

    @Override
    public boolean isFinished() {
        return intake.robotHasCoral(); // the command is finished when the intake has the note
    }
}
