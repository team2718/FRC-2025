package frc.robot.commands.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean autonomousFinished = false;

    private double xDistance = 0.0;
    private double yDistance = 0.0;
    private double thetaDistance = 0.0;

    private Timer timer;

    public AutoScoringCommand(SuperSystem supersystem, SwerveSubsystem swerve, ArmSubsystem arm, ElevatorSubsystem elevator, EndEffectorSubsystem effector, Vision vision) {
        this.supersystem = supersystem;
        this.arm = arm;
        this.vision = vision;
        this.swerve = swerve;
        this.effector = effector;

        xPID.setTolerance(0.3);
        yPID.setTolerance(0.3);
        thetaPID.setTolerance(1.0);

        timer = new Timer();

        addRequirements(supersystem, swerve, arm, elevator);
    }

    public AutoScoringCommand auto() {
        autonomousFinished = true;
        return this;
    }

    @Override
    public void initialize() {
        xDistance = 0.0;
        yDistance = 0.0;
        thetaDistance = 0.0;

        xPID.reset();
        yPID.reset();
        thetaPID.reset();

        complete = false;

        timer.reset();
    }

    @Override
    public void execute() {

        effector.setEndEffector(0);

        double ySetpoint = Constants.AutoAlignConstants.leftBranchToCamera;
        if (!supersystem.isScoringLeft()) {
            ySetpoint += Constants.AutoAlignConstants.distanceBetweenBranches; // Distance between branches
        }

        // Note: X and Y are swapped because the camera is mounted perpendicular to the robot's forward direction
        xDistance = vision.getVisionY();
        yDistance = -vision.getVisionX();
        thetaDistance = vision.getVisionTheta();

        double x = MathUtil.clamp(xPID.calculate(xDistance, ySetpoint), -0.5, 0.5);
        double y = MathUtil.clamp(yPID.calculate(yDistance, -Constants.AutoAlignConstants.reefWallToCamera), -0.5, 0.5);
        double theta = MathUtil.clamp(thetaPID.calculate(thetaDistance, 0.0), -0.4, 0.4);

        if (complete || (xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint())) {
            supersystem.setScoring();

            if (complete || (arm.atPosition() && arm.getArmAngle() < 60)) {
                complete = true;
                timer.start();
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
        swerve.drive(new Translation2d(0, 0), 0, false);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (autonomousFinished) {
            return (complete && timer.get() > 1.0); // Complete the routine once the robot has scored and moved away
        } else {
            return false;
        }
    }

}
