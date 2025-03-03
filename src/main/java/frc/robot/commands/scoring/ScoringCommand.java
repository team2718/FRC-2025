package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.*;

public class ScoringCommand extends Command {
    private final SuperSystem supersystem;

    public ScoringCommand(SuperSystem supersystem, ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.supersystem = supersystem;

        addRequirements(supersystem, arm, elevator);
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
