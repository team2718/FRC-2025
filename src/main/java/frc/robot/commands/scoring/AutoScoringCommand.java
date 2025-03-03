package frc.robot.commands.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoScoringCommand extends Command {
    private final SuperSystem supersystem;
    private final SwerveSubsystem swerve;
    private final ArmSubsystem arm;
    private final EndEffectorSubsystem effector;
    private final Vision vision;

    private final PIDController xPID = new PIDController(3.0, 0.01, 0.0);
    private final PIDController yPID = new PIDController(3.0, 0.01, 0.0);
    private final PIDController thetaPID = new PIDController(0.15, 0.001, 0.0);

    private boolean complete = false;

    public AutoScoringCommand(SuperSystem supersystem, SwerveSubsystem swerve, ArmSubsystem arm, ElevatorSubsystem elevator, EndEffectorSubsystem effector, Vision vision) {
        this.supersystem = supersystem;
        this.arm = arm;
        this.vision = vision;
        this.swerve = swerve;
        this.effector = effector;

        xPID.setTolerance(0.1);
        yPID.setTolerance(0.1);
        thetaPID.setTolerance(1.0);

        addRequirements(supersystem, swerve, arm, elevator);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        thetaPID.reset();
        complete = false;
    }

    @Override
    public void execute() {

        effector.setEndEffector(0);

        double ySetpoint = Constants.AutoAlignConstants.leftBranchToCamera;
        if (!supersystem.isScoringLeft()) {
            ySetpoint += Constants.AutoAlignConstants.distanceBetweenBranches; // Distance between branches
        }

        double x = MathUtil.clamp(xPID.calculate(vision.getVisionY(), ySetpoint), -0.5, 0.5);
        double y = -MathUtil.clamp(yPID.calculate(vision.getVisionX(), Constants.AutoAlignConstants.reefWallToCamera), -0.5, 0.5);
        double theta = MathUtil.clamp(thetaPID.calculate(vision.getVisionTheta(), 0.0), -0.4, 0.4);

        if (complete || (xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint())) {
            supersystem.setScoring();

            if (complete || (arm.atPosition() && arm.getArmAngle() < 75)) {
                complete = true;
                y = -0.8;
                x = 0;
                theta = 0;
                effector.setEndEffector(-0.1);
            }
        } else {
            supersystem.setMoveAuto();
        }

        swerve.drive(new Translation2d(x, y), theta, false);
    }

    @Override
    public void end(boolean interuppted) {
        supersystem.setIntake();
        effector.setEndEffector(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
