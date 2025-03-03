// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Competition mode will remove or reduce feedback to minmize unnecessary
  // computation
  public static final boolean COMPETITION_MODE = false;

  // Robot constants
  public static final double ROBOT_MASS = Units.lbsToKilograms(130);
  public static final Translation3d CENTER_OF_GRAVITY = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final Matter CHASSIS = new Matter(CENTER_OF_GRAVITY, ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(12.9);

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    public static final double SPEED_MULTIPLIER = 0.6;
    public static final double ROTATION_MULTIPLIER = 0.6;
    public static final double DEADBAND = 0.3;
  }

  public static final class IntakeConstants {
    public static final int intakemotorID = 10;
    public static final int intakeLimitSwitchChannel = 5;
    public static final int flappermotorID = 16;
  }

  public static final class ArmConstants {
    public static final int armMotorID = 13;
  }

  public static final class EndEffectorConstants {
    public static final int endeffectormotor1ID = 14;
  }

  public static final class ElevatorConstants {
    public static final int elevatormotor1ID = 11;
    public static final int elevatormotor2ID = 12;
    public static final double elevatorTolerance = 1;

  }

  public static final class ClimberConstants {
    public static final int climbermotorID = 15;
  }

  public static final class AutoAlignConstants {
    public static final double leftBranchToCamera = -0.05; // meters
    public static final double distanceBetweenBranches = Units.inchesToMeters(12.94);
    public static final double reefWallToCamera = 0.84; // meters
  }
}