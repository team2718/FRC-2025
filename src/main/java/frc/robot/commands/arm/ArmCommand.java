package frc.robot.commands.arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {
    private final ArmSubsystem arm;

public ArmCommand(ArmSubsystem arm, double speed) {
    this.arm = arm;
    addRequirements(arm);
}

@Override
    public void initialize() {
        
    }
    
@Override
    public void execute() {
        arm.setSpeed(0.8);
    }
    
@Override
    public void end(boolean interuppted) {
        arm.setSpeed(0);
     
    }
@Override
    public boolean isFinished() {
       return true;
    }
}
