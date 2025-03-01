package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;


public ClimberSubsystem() {
    climberMotor = new SparkMax(Constants.ClimberConstants.climbermotorID, MotorType.kBrushless);
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.idleMode(IdleMode.kBrake);
    climberConfig.inverted(false);
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void setClimber(double speed) {
    climberMotor.set(speed);
}
}
