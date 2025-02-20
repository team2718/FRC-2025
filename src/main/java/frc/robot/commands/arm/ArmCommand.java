package frc.robot.commands.arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {
    private final ArmSubsystem arm;
    private double speed;

public ArmCommand(ArmSubsystem arm, double speed) {
    this.arm = arm;
    addRequirements(arm);
    this.speed = speed;
}

@Override
    public void initialize() {
        
    }
    
@Override
    public void execute() {
        arm.setArm(speed);
    }
    
@Override
    public void end(boolean interuppted) {
        arm.setArm(0);
     
    }
@Override
    public boolean isFinished() {
       return false;
    }
}
