package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class AutonomousWaitForFeed extends Command{

    private final EndEffectorSubsystem effector;
    private final SuperSystem supersystem;

    public AutonomousWaitForFeed(SuperSystem supersystem, EndEffectorSubsystem effector) {
        this.effector = effector;
        this.supersystem = supersystem;
        addRequirements(effector, supersystem);
    }

    @Override
    public void execute() {
        supersystem.setIntake();
        effector.setIntake();
    }

    @Override
    public void end(boolean interuppted) {
        effector.setHold();
    }

    @Override
    public boolean isFinished() {
        return effector.hasCoral();
    }


}
