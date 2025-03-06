// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.ScoringPositions;
import frc.robot.commands.intake.*;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.elevator.SSElevatorCommand;
import frc.robot.commands.endeffector.*;
import frc.robot.commands.scoring.AutoScoringCommand;
import frc.robot.commands.scoring.ScoringCommand;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController secondXbox = new
  CommandXboxController(1);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private final Vision vision = new Vision(drivebase.getSwerveDrive().field);

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final EndEffectorSubsystem endeffector = new EndEffectorSubsystem();
  private final SuperSystem supersystem = new SuperSystem(arm, elevator);
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final Timer matchTimer = new Timer();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MULTIPLIER)
      .scaleRotation(OperatorConstants.ROTATION_MULTIPLIER)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
          () -> -driverXbox.getRightY())
      .headingWhile(true);

  // Drive with right-stick direct angle control
  // Command driveFieldOrientedDirectAngle =
  // drivebase.driveFieldOriented(driveDirectAngle);

  // Drive with right-stick angular velocity control
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private SendableChooser<String> autoChooser = new SendableChooser<String>();

  private final ScoringCommand score = new ScoringCommand(supersystem, arm, elevator);
  private final AutoScoringCommand autoScore = new AutoScoringCommand(supersystem, drivebase, arm, elevator,
      endeffector, vision);

  private final ClimberCommand climbout = new ClimberCommand(climber, 0.5);
  private final ClimberCommand climbin = new ClimberCommand(climber, -0.5);

  private final SSElevatorCommand elevatorL4Command = new SSElevatorCommand(supersystem, ScoringPositions.L4);
  private final SSElevatorCommand elevatorL3Command = new SSElevatorCommand(supersystem, ScoringPositions.L3);
  private final SSElevatorCommand elevatorL2Command = new SSElevatorCommand(supersystem, ScoringPositions.L2);
  private final SSElevatorCommand elevatorL1Command = new SSElevatorCommand(supersystem, ScoringPositions.L1);

  private int position = 4;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    supersystem.setScoringPosition(ScoringPositions.L4);

    NamedCommands.registerCommand("AutoScore",
        new AutoScoringCommand(supersystem, drivebase, arm, elevator, endeffector, vision).auto());

    ScoringPositions[] positions = { ScoringPositions.L1, ScoringPositions.L2, ScoringPositions.L3,
        ScoringPositions.L4 };
    String[] leftright = { "Left", "Right" };

    // Register commands for setting scoring position and side in PathPlanner
    // Commands will look like "L1-Left", "L1-Right", "L2-Left", "L2-Right", etc.
    for (ScoringPositions pos : positions) {
      for (String lr : leftright) {
        NamedCommands.registerCommand(pos.name() + "-" + lr, Commands.runOnce(() -> {
          supersystem.setScoringPosition(pos);
          if (lr.equals("Left")) {
            supersystem.setScoringLeft();
          } else {
            supersystem.setScoringRight();
          }
        }));
      }
    }

    autoChooser.setDefaultOption("Leave", "Leave");
    autoChooser.addOption("Preload Proc", "Preload Proc");
    autoChooser.addOption("Preload Middle", "Preload Middle");
    autoChooser.addOption("Preload NonProc", "Preload NonProc");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    matchTimer.reset();

    // Start match time on teleop start
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
      matchTimer.reset();
      matchTimer.start();
    }));

    // Stop match time on end of match
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
      matchTimer.stop();
    }));

    configureBindings();
  }

  private void configureBindings() {

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverXbox.leftTrigger()
        .whileTrue(new IntakeCommand(intake, 0.45).alongWith(new EndEffectorCommand(endeffector, 0.9)));
    driverXbox.leftBumper()
        .whileTrue(new IntakeCommand(intake, -0.5).alongWith(new EndEffectorCommand(endeffector, -0.15)));

    driverXbox.rightTrigger().whileTrue(autoScore);
    driverXbox.rightBumper().whileTrue(score);

   // driverXbox.start().whileTrue(climbin);
   // driverXbox.back().whileTrue(climbout);

    //driverXbox.povUp().onTrue(Commands.runOnce(() -> adjPosition(1))).debounce(0.4);
    //driverXbox.povDown().onTrue(Commands.runOnce(() -> adjPosition(-1))).debounce(0.4);

   // driverXbox.povRight().onTrue(Commands.runOnce(() -> supersystem.setScoringLeft())).debounce(0.4);
   // driverXbox.povLeft().onTrue(Commands.runOnce(() -> supersystem.setScoringRight())).debounce(0.4);

    // sets scoring positions 
    secondXbox.rightBumper().onTrue(Commands.runOnce(() -> supersystem.setScoringLeft())).debounce(0.4);
    secondXbox.leftBumper().onTrue(Commands.runOnce(() -> supersystem.setScoringRight())).debounce(0.4);

    secondXbox.y().whileTrue(elevatorL4Command);
    secondXbox.x().whileTrue(elevatorL3Command);
    secondXbox.b().whileTrue(elevatorL2Command);
    secondXbox.a().whileTrue(elevatorL1Command);

    //secondXbox.rightBumper().whileTrue(climbin.alongWith(Commands.runOnce(supersystem::setClimbState)));
    //secondXbox.leftBumper().whileTrue(climbout.alongWith(Commands.runOnce(supersystem::setClimbState)));

    elevator.resetPosition();
  }

  public void periodic() {
    SmartDashboard.putNumber("Match Time", (2 * 60 + 15) - matchTimer.get());
    updateOdometry();
    
    
  }

  private void adjPosition(int change) {
    position += change;

    if (position < 1) {
      position = 1;
    } else if (position > 4) {
      position = 4;
    }

    switch (position) {
      case 1:
        supersystem.setScoringPosition(ScoringPositions.L1);
        break;
      case 2:
        supersystem.setScoringPosition(ScoringPositions.L2);
        break;
      case 3:
        supersystem.setScoringPosition(ScoringPositions.L3);
        break;
      case 4:
        supersystem.setScoringPosition(ScoringPositions.L4);
        break;
    }

    elevator.resetProfilePID();
    arm.resetProfilePID();
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand(autoChooser.getSelected());
  }

  public void updateOdometry() {
    vision.updatePoseEstimation(drivebase.getSwerveDrive());

    SmartDashboard.putNumber("X", vision.getVisionX());
    SmartDashboard.putNumber("Y", vision.getVisionY());
    SmartDashboard.putNumber("Theta", vision.getVisionTheta());
  }

  public void resetProfilePIDs() {
    elevator.resetProfilePID();
    arm.resetProfilePID();
  }
}