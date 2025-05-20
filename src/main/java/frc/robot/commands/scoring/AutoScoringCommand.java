package frc.robot.commands.scoring;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
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

    private final PIDController xPID = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yPID = new PIDController(2.0, 0.0, 0.0);
    private final PIDController thetaPID = new PIDController(0.09, 0.0, 0.0);

    private boolean complete = false;
    private boolean elevatorRaised = false;

    private double xDistance = 0.0;
    private double yDistance = 0.0;
    private double thetaDistance = 0.0;

    private Timer timer;

    List<AprilTag> reefTags = new ArrayList<>();
    int[] reefTagIDs = {6,7,8,9,10,11,  17,18,19,20,21,22};

    public AutoScoringCommand(SuperSystem supersystem, SwerveSubsystem swerve, ArmSubsystem arm, ElevatorSubsystem elevator, EndEffectorSubsystem effector, Vision vision) {
        this.supersystem = supersystem;
        this.arm = arm;
        this.vision = vision;
        this.swerve = swerve;
        this.effector = effector;

        xPID.setTolerance(0.4);
        yPID.setTolerance(0.4);
        thetaPID.setTolerance(2.0);

        timer = new Timer();

        for (AprilTag tag : Vision.fieldLayout.getTags()) {
            for (int id : reefTagIDs) {
                if (tag.ID == id) { // Reef tags
                    reefTags.add(tag);
                }
            }
        }

        addRequirements(supersystem, swerve, arm, elevator);
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
        elevatorRaised = false;

        timer.reset();
    }

    private void runNoVision() {
        AprilTag nearestTag = null;
        double minDistance = Double.MAX_VALUE;
        for (AprilTag tag : reefTags) {
            double distance = swerve.getPose().getTranslation().getDistance(new Translation2d(tag.pose.getX(), tag.pose.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                nearestTag = tag;
            }
        }

        // Once we're close enough, go ahead and bring up the elevator
        if (elevatorRaised || minDistance < Constants.AutoAlignConstants.minDistanceToRaiseElevator) {
            elevatorRaised = true;
            supersystem.setMoveAuto();
        } else {
            supersystem.setIntake();
        }

        // If we can't see any tags, just drive towards the nearest tag
        // Drive 1.3 meters away (this is the diagonal width of the robot to prevent hitting the reef)
        double targetXPosition = nearestTag.pose.getX() + 1.3 * Math.cos(nearestTag.pose.getRotation().getZ());
        double targetYPosition = nearestTag.pose.getY() + 1.3 * Math.sin(nearestTag.pose.getRotation().getZ());
        xDistance = swerve.getPose().getX() - targetXPosition;
        yDistance = swerve.getPose().getY() - targetYPosition;
        
        double desiredRotation = nearestTag.pose.getRotation().getZ() * 180 / Math.PI + 90; // -90 because the scoring is on the robot's left side
        // Wrap to -180 to 180
        if (desiredRotation > 180) {
            desiredRotation -= 360;
        } else if (desiredRotation < -180) {
            desiredRotation += 360;
        }

        double x = MathUtil.clamp(xPID.calculate(xDistance, 0.0), -1.0, 1.0);
        double y = MathUtil.clamp(yPID.calculate(yDistance, 0.0), -1.0, 1.0);
        double theta = MathUtil.clamp(thetaPID.calculate(swerve.getPose().getRotation().getDegrees(), desiredRotation), -2.5, 2.5);

        swerve.drive(new Translation2d(x, y), theta, true);
    }

    @Override
    public void execute() {

        effector.setHold();

        // If we can't see a good april tag, just drive to the nearest reef face
        if (!vision.visionCanSeeTarget()) {
            supersystem.setIntake();
            runNoVision();
            return;
        }

        xDistance = vision.getVisionY();
        yDistance = vision.getVisionX();
        thetaDistance = vision.getVisionTheta();

        double xSetpoint = Constants.AutoAlignConstants.leftBranchToCamera;
        if (!supersystem.isScoringLeft()) {
            xSetpoint += Constants.AutoAlignConstants.distanceBetweenBranches; // Distance between branches
        }

        double x = MathUtil.clamp(xPID.calculate(xDistance, xSetpoint), -1.0, 1.0);
        double y = -MathUtil.clamp(yPID.calculate(yDistance, supersystem.getScoringPosition().getReefDistance()), -1.0, 1.0);
        double theta = MathUtil.clamp(thetaPID.calculate(thetaDistance, 5), -1.0, 1.0);

        if (complete || (xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint())) {
            elevatorRaised = true;
            supersystem.setScoring();

            // The scoring is "complete" when the arm is at a scoring position
            if (complete || (arm.atPosition() && arm.getArmTargetPosition() < (arm.safeRaisingPosition - 5))) {
                complete = true;
                timer.start();
                effector.setScore();
            }
        } else if(elevatorRaised || yDistance < Constants.AutoAlignConstants.minDistanceToRaiseElevator) {
            elevatorRaised = true;
            supersystem.setMoveAuto();
        } else {
            supersystem.setIntake();
        }

        swerve.drive(new Translation2d(x, y), theta, false);
    }

    @Override
    public void end(boolean interuppted) {
        supersystem.setIntake();
        effector.setHold();
        swerve.drive(new Translation2d(0, 0), 0, false);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        // if (autonomousFinished) {
        return (complete && timer.get() > 0.7); // Complete the routine once the robot has scored and moved away
        // } else {
        //     return false;
        // }
    }

}
