package frc.robot.subsystems.climber;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    SparkMaxConfig climberConfig;

    Alert climberMotorAlert;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(Constants.ClimberConstants.climbermotorID, MotorType.kBrushless);

        climberConfig = new SparkMaxConfig();
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.inverted(false);
        climberConfig.smartCurrentLimit(20);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberMotorAlert = new Alert("Motor \"" + "Climber Motor" + "\" is not connected!", AlertType.kError);
    }

    public void setClimber(double speed) {
        climberMotor.set(speed);
    }

    // alerts us if the climber motor is not detected
    public Alert setClimberMotorAlert() {
        if (climberMotor.getBusVoltage() > 0) {
            climberMotorAlert.set(false); 
        } else {
            climberMotorAlert.set(true);
        }
        
        return climberMotorAlert;
    }

}