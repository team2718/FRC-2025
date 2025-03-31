package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax engageMotor;
    private final SparkMax winchMotor;

    SparkMaxConfig engageConfig;
    SparkMaxConfig winchConfig;

    boolean isWinchRunning = false;

    Alert engageMotorAlert;
    Alert winchMotorAlert;

    public ClimberSubsystem() {
        winchMotor = new SparkMax(Constants.ClimberConstants.winchmotorID, MotorType.kBrushless);
        engageMotor = new SparkMax(Constants.ClimberConstants.climberengagemotorID, MotorType.kBrushless);

        engageConfig = new SparkMaxConfig();
        engageConfig.idleMode(IdleMode.kBrake);
        engageConfig.inverted(false);
        engageConfig.smartCurrentLimit(20);

        winchConfig = new SparkMaxConfig();
        winchConfig.idleMode(IdleMode.kCoast);
        winchConfig.inverted(false);
        winchConfig.smartCurrentLimit(30);

        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        engageMotor.configure(engageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        engageMotorAlert = new Alert("Motor \"" + "Engage Motor" + "\" is faulting!", AlertType.kError);
        winchMotorAlert = new Alert("Motor \"" + "Engage Motor" + "\" is faulting!", AlertType.kError);

        engageMotor.getEncoder().setPosition(0.0);
    }

    @Override
    public void periodic() {
        setAlerts();
        SmartDashboard.putNumber("Climber Engage Position", getEngagePosition());
    }

    public double getEngagePosition() {
        return engageMotor.getEncoder().getPosition(); // Returns the position of the engage motor
    }

    public void setWinchMotor(double voltage) {
        if (!isWinchRunning) {
            winchConfig.idleMode(IdleMode.kBrake);
            engageConfig.idleMode(IdleMode.kCoast);

            winchMotor.configure(winchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            engageMotor.configure(engageConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            isWinchRunning = true;
        }
        winchMotor.setVoltage(voltage);
        engageMotor.setVoltage(0);
    }

    public void setEngageMotor(double voltage) {
        if (isWinchRunning) {
            winchConfig.idleMode(IdleMode.kCoast);
            engageConfig.idleMode(IdleMode.kBrake);

            winchMotor.configure(winchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            engageMotor.configure(engageConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            isWinchRunning = false;
        }
        engageMotor.setVoltage(voltage);
        winchMotor.setVoltage(0);
    }

    // alerts us if the arm motor is not detected
    public void setAlerts() {
        engageMotorAlert.set(engageMotor.hasActiveFault());
        winchMotorAlert.set(winchMotor.hasActiveFault());
    }
}