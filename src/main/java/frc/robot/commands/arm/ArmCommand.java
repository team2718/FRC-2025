package frc.robot.commands.arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {
    private final ArmSubsystem arm;
    private double position;

public ArmCommand(ArmSubsystem arm, double position) {
    this.arm = arm;
    addRequirements(arm);
    this.position = position;
}

@Override
    public void initialize() {
        
    }
    
@Override
    public void execute() {
        arm.setArmTargetPosition(position);
    }
    
@Override
    public void end(boolean interuppted) {
        
     
    }
@Override
    public boolean isFinished() {
       return false;
    }
}
