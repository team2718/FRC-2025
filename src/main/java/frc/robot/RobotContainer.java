// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//subsystems
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.ScoringPositions;
//commands
import frc.robot.commands.elevator.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.endeffector.*;
import frc.robot.commands.scoring.ScoringCommand;
import java.io.File;

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
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      // "swerve"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final EndEffectorSubsystem endeffector = new EndEffectorSubsystem();
  private final SuperSystem supersystem = new SuperSystem(arm, elevator);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      // () -> driverXbox.getLeftY() * -1,
      // () -> driverXbox.getLeftX() * -1)
      // .withControllerRotationAxis(driverXbox::getRightX)
      // .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(OperatorConstants.SPEED_MULTIPLIER)
      // .scaleRotation(OperatorConstants.ROTATION_MULTIPLIER)
      // .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      // .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
          // () -> -driverXbox.getRightY())
      // .headingWhile(true);

  // Drive with right-stick direct angle control
//  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Drive with right-stick angular velocity control
 // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private SendableChooser<String> autoChooser = new SendableChooser<String>();
  private ShuffleboardTab tab = Shuffleboard.getTab("auto chooser");

  
  private final ElevatorCommand elevatorL3Position = new ElevatorCommand(elevator, 13.45);
  private final ElevatorCommand elevatorbottomPosition = new ElevatorCommand(elevator, 0.1);

  private final IntakeCommand runIntake = new IntakeCommand(intake, 0.8);

  private final ArmCommand runArm90 = new ArmCommand(arm, 90);
  private final ArmCommand runArm60 = new ArmCommand(arm, 40.52);

  private final EndEffectorCommand runEffector = new EndEffectorCommand(endeffector, 0.6);
  private final EndEffectorCommand outtakeEffector = new EndEffectorCommand(endeffector, -0.6);

  private final ScoringCommand score = new ScoringCommand(supersystem);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
   // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    // driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.leftBumper().whileTrue(runIntake);
    
    driverXbox.b().whileTrue(runArm60);
    driverXbox.x().whileTrue(runArm90);
    
    driverXbox.a().whileTrue(elevatorbottomPosition);
    driverXbox.y().whileTrue(elevatorL3Position);

    driverXbox.rightBumper().whileTrue(runEffector);
    driverXbox.rightTrigger().whileTrue(outtakeEffector);

    driverXbox.leftTrigger().whileTrue(score);
    
 }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return drivebase.getAutonomousCommand("Move 1m");
   return null;
  }

  public void setDriveMode() {
    configureBindings();
  }

  // public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  // }
}