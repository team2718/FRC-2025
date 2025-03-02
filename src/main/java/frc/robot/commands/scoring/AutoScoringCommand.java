package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoScoringCommand extends Command {
    private final SuperSystem supersystem;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final Vision vision;

    public AutoScoringCommand(SuperSystem supersystem, ArmSubsystem arm, ElevatorSubsystem elevator, Vision vision) {
        this.supersystem = supersystem;
        this.arm = arm;
        this.elevator = elevator;
        this.vision = vision;

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
