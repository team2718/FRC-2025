package frc.robot.commands.intake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command{
    private final IntakeSubsystem intake;

public IntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.setSpeed(0.8);
    }

    @Override
    public void end(boolean interuppted) {
        intake.setSpeed(0);
     
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote(); // the command is finished when the intake has the note
    }
}

