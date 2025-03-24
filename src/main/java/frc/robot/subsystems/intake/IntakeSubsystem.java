package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// one motor, sensor to detect pieces

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax flappermotor;
    SparkMaxConfig flapperConfig;

    Alert intakeMotorAlert;
    Alert flapperMotorAlert;

    public IntakeSubsystem() {
        flappermotor = new SparkMax(Constants.IntakeConstants.flappermotorID, MotorType.kBrushless);
        SparkMaxConfig flapperConfig = new SparkMaxConfig();
        flapperConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(2);
        flappermotor.configure(flapperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flapperMotorAlert = new Alert("Motor \"" + "Flapper Motor" + "\" is not connected!", AlertType.kError);
    }

    @Override
    public void periodic() {
        flappermotor.set(0.15);
    }

    public void setFlapperBrake(boolean brake) {
        flapperConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        flappermotor.configure(flapperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveFlapper(double speed) {
        flappermotor.set(speed);
    }

    // alerts us if the flapper motor is not detected
    public Alert setFlapperMotorAlert() {
        if (flappermotor.getBusVoltage() > 0) {
            flapperMotorAlert.set(false); 
        } else {
            flapperMotorAlert.set(true);
        }
        
        return flapperMotorAlert;
    }

}