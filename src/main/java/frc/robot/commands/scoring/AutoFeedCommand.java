package frc.robot.commands.scoring;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoFeedCommand extends Command {
    private final SuperSystem supersystem;
    private final SwerveSubsystem swerve;
    private final EndEffectorSubsystem effector;
    private final Supplier<ChassisSpeeds> velocitySupplier;
    private final PIDController thetaPID = new PIDController(0.09, 0.0, 0.0);

    List<AprilTag> feedTags = new ArrayList<>();

    public AutoFeedCommand(SuperSystem supersystem, SwerveSubsystem swerve, ArmSubsystem arm,
            ElevatorSubsystem elevator, EndEffectorSubsystem effector,
            Supplier<ChassisSpeeds> velocitySupplier) {
        this.supersystem = supersystem;
        this.swerve = swerve;
        this.effector = effector;
        this.velocitySupplier = velocitySupplier;

        thetaPID.setTolerance(1.0);

        for (AprilTag tag : Vision.fieldLayout.getTags()) {
            if (tag.ID == 1 || tag.ID == 2 || tag.ID == 12 || tag.ID == 13) { // feed station tags
                feedTags.add(tag);
            }
        }

        addRequirements(supersystem, swerve, arm, elevator, effector);
    }

    @Override
    public void initialize() {
        thetaPID.reset();
    }

    @Override
    public void execute() {

        supersystem.setIntake();
        effector.setIntake();

        // Find the nearest tag
        AprilTag nearestTag = null;
        double minDistance = Double.MAX_VALUE;
        for (AprilTag tag : feedTags) {
            double distance = swerve.getPose().getTranslation()
                    .getDistance(new Translation2d(tag.pose.getX(), tag.pose.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                nearestTag = tag;
            }
        }

        ChassisSpeeds velocity = velocitySupplier.get();

        double desiredRotation = nearestTag.pose.getRotation().getZ() * 180 / Math.PI - 90; // -90 because the scoring
                                                                                            // is on the robot's left
                                                                                            // side
        // Wrap to -180 to 180
        if (desiredRotation > 180) {
            desiredRotation -= 360;
        } else if (desiredRotation < -180) {
            desiredRotation += 360;
        }

        SmartDashboard.putNumber("AutoFeed/NearestTagID", nearestTag.ID);
        SmartDashboard.putNumber("AutoFeed/NearestTagRotation", desiredRotation);
        SmartDashboard.putNumber("Robot Rotation", swerve.getPose().getRotation().getDegrees());

        // Use nearest tag to set desired heading
        // -90 because the scoring is on the robot's left side
        velocity.omegaRadiansPerSecond = MathUtil.clamp(
                thetaPID.calculate(swerve.getPose().getRotation().getDegrees(), desiredRotation), -2.5, 2.5);

        SmartDashboard.putNumber("AutoFeed/VelocityOmega", velocity.omegaRadiansPerSecond);

        swerve.driveFieldOriented(velocity);
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
