// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.elevator.SSElevatorCommand;
import frc.robot.commands.scoring.AutoFeedCommand;
import frc.robot.commands.scoring.AutoScoringCommand;
import frc.robot.commands.scoring.AutonomousWaitForFeed;
import frc.robot.commands.scoring.ScoringCommand;
import java.io.File;

import javax.security.auth.callback.NameCallback;

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
  private final CommandXboxController secondXbox = new CommandXboxController(1);

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

  private boolean elevatorEnabled = true;
  private boolean armEnabled = true;
  private boolean driveEnabled = true;
  private boolean endeffectorEnabled = true;
  private boolean allEnabled = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    supersystem.setScoringPosition(ScoringPositions.L4);

    NamedCommands.registerCommand("AutoScore",
        new AutoScoringCommand(supersystem, drivebase, arm, elevator, endeffector, vision).auto());

    NamedCommands.registerCommand("WaitForFeed",
        new AutonomousWaitForFeed(supersystem, endeffector));

    NamedCommands.registerCommand("RaiseElevator", Commands.runOnce(() -> {
      supersystem.setMoveAuto();
    }, supersystem));

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
    autoChooser.addOption("NonProc 2 Piece", "NonProc 2 Piece");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    matchTimer.reset();

    // Start match time on autonomous start
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
      matchTimer.reset();
      matchTimer.start();
    }));

    // Start match time on teleop start
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
      matchTimer.reset();
      matchTimer.start();
    }));

    // Stop match time on end of match
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
      matchTimer.reset();
      matchTimer.stop();
    }));

    configureBindings();
  }

  private void configureBindings() {

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverXbox.leftTrigger().whileTrue(
        new AutoFeedCommand(supersystem, drivebase, arm, elevator, endeffector, vision, driveAngularVelocity));
    driverXbox.leftBumper().whileTrue(Commands.runEnd(() -> {
      endeffector.setOuttake();
    }, () -> {
      endeffector.setHold();
    }, endeffector));

    driverXbox.b().whileTrue(Commands.runEnd(() -> {
      endeffector.setDealgify();
    }, () -> {
      endeffector.setHold();
    }, endeffector));

    driverXbox.rightTrigger().whileTrue(autoScore);
    driverXbox.rightBumper().whileTrue(score);

    // driverXbox.start().whileTrue(climbin);
    // driverXbox.back().whileTrue(climbout);

    // sets scoring positions
    secondXbox.leftBumper().onTrue(Commands.runOnce(() -> supersystem.setScoringLeft())).debounce(0.4);
    secondXbox.rightBumper().onTrue(Commands.runOnce(() -> supersystem.setScoringRight())).debounce(0.4);

    secondXbox.y().whileTrue(elevatorL4Command);
    secondXbox.x().whileTrue(elevatorL3Command);
    secondXbox.b().whileTrue(elevatorL2Command);
    secondXbox.a().whileTrue(elevatorL1Command);

    // secondXbox.rightBumper().whileTrue(climbin.alongWith(Commands.runOnce(supersystem::setClimbState)));
    // secondXbox.leftBumper().whileTrue(climbout.alongWith(Commands.runOnce(supersystem::setClimbState)));
  }

  private int stateToButtonBox() {
    int[] buttonBoxLEDs = { 0, 0, 0, 0, 0, 0, 0, 0 };

    switch (supersystem.getScoringPosition()) {
      case L1:
        buttonBoxLEDs[0] = 1;
        break;
      case L2:
        buttonBoxLEDs[1] = 1;
        break;
      case L3:
        buttonBoxLEDs[2] = 1;
        break;
      case L4:
        buttonBoxLEDs[3] = 1;
        break;
    }

    if (supersystem.isScoringLeft()) {
      buttonBoxLEDs[4] = 1;
      buttonBoxLEDs[5] = 1;
    } else {
      buttonBoxLEDs[6] = 1;
      buttonBoxLEDs[7] = 1;
    }

    return buttonBoxLEDs[0] * 1 + buttonBoxLEDs[1] * 2 + buttonBoxLEDs[2] * 4 + buttonBoxLEDs[3] * 8
        + buttonBoxLEDs[4] * 16
        + buttonBoxLEDs[5] * 32 + buttonBoxLEDs[6] * 64 + buttonBoxLEDs[7] * 128;
  }

  public void periodic() {
    secondXbox.setRumble(RumbleType.kBothRumble, stateToButtonBox() / 255.0);
    // secondXbox.setRumble(RumbleType.kRightRumble, 1); // Set it to anything to
    // let the button box know it's a "real" command

    int leftTrigger = (int) (secondXbox.getHID().getLeftTriggerAxis() * 32);

    elevatorEnabled = (leftTrigger & 0b1) == 0;
    armEnabled = (leftTrigger & 0b10) == 0;
    driveEnabled = (leftTrigger & 0b100) == 0;
    endeffectorEnabled = (leftTrigger & 0b1000) == 0;
    allEnabled = (leftTrigger & 0b10000) == 0;

    if (!allEnabled) {
      elevatorEnabled = false;
      armEnabled = false;
      driveEnabled = false;
      endeffectorEnabled = false;
    }

    SmartDashboard.putBoolean("Enabled/Elevator", elevatorEnabled);
    SmartDashboard.putBoolean("Enabled/Arm", armEnabled);
    SmartDashboard.putBoolean("Enabled/Drive", driveEnabled);
    SmartDashboard.putBoolean("Enabled/EndEffector", endeffectorEnabled);
    SmartDashboard.putBoolean("Enabled/All", allEnabled);

    SmartDashboard.putBoolean("Vision Can See Target", vision.visionCanSeeTarget());

    elevator.setEnabled(elevatorEnabled);
    arm.setEnabled(armEnabled);
    drivebase.setEnabled(driveEnabled);
    endeffector.setEnabled(endeffectorEnabled);

    supersystem.setDialPosition(secondXbox.getHID().getRightTriggerAxis());

    if (secondXbox.getHID().getLeftBumperButton() || secondXbox.getHID().getRightBumperButton()) {
      supersystem.setDialContrlArm(true);
    } else {
      supersystem.setDialContrlArm(false);
    }

    if (DriverStation.isAutonomous()) {
      SmartDashboard.putNumber("Match Time", (15) - matchTimer.get());
    }

    if (DriverStation.isTeleop()) {
      SmartDashboard.putNumber("Match Time", (2 * 60 + 15) - matchTimer.get());
    }

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
    arm.resetProfilePID();
  }
}