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
    private final SparkMax intakemotor;
    private final SparkMax flappermotor;
    private final DigitalInput IntakeLimitSwitch;
    public boolean flapperset = false;

    SparkMaxConfig flapperConfig;

    Alert intakeMotorAlert;
    Alert flapperMotorAlert;

    public IntakeSubsystem() {
        intakemotor = new SparkMax(Constants.IntakeConstants.intakemotorID, MotorType.kBrushless);
        flappermotor = new SparkMax(Constants.IntakeConstants.flappermotorID, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        flapperConfig = new SparkMaxConfig();

        intakeConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        flapperConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(2);

        intakemotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flappermotor.configure(flapperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeLimitSwitch = new DigitalInput(Constants.IntakeConstants.intakeLimitSwitchChannel);

        intakeMotorAlert = new Alert("Motor \"" + "Intake Motor" + "\" is not connected!", AlertType.kError);
        flapperMotorAlert = new Alert("Motor \"" + "Flapper Motor" + "\" is not connected!", AlertType.kError);
    }

    @Override
    public void periodic() {
        flappermotor.set(0.1);
    }

    public void setFlapperBrake(boolean brake) {
        flapperConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        flappermotor.configure(flapperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double power) {
        intakemotor.set(power);
    }

    public void stopIntake() {
        intakemotor.set(0);
    }

    // sets intake in commmand
    public void setIntake(double speed) {
        intakemotor.set(speed);

    }

    public void moveFlapper(double speed) {
        flappermotor.set(speed);
    }

    // testing if the limit switch sees the note or not
    public boolean hasNote() {
        return !IntakeLimitSwitch.get();
    }

    // alerts us if the intake motor is not detected
    public Alert setIntakeMotorAlert() {
        if (intakemotor.getBusVoltage() > 0) {
            intakeMotorAlert.set(false); 
        } else {
            intakeMotorAlert.set(true);
        }
        
        return intakeMotorAlert;
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