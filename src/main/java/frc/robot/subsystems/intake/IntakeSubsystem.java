package frc.robot.subsystems.intake;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;

// one motor, sensor to detect pieces

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakemotor; 
    private final DigitalInput IntakeLimitSwitch;


    public IntakeSubsystem() {
        intakemotor = new SparkMax(Constants.IntakeConstants.intakemotorID, MotorType.kBrushless);
        

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        

        intakeConfig.inverted(false).idleMode(IdleMode.kBrake);
        

        intakemotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       

        IntakeLimitSwitch = new DigitalInput(Constants.IntakeConstants.intakeLimitSwitchChannel);
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

    // testing if the limit switch sees the note or not
    public boolean hasNote() {
        return !IntakeLimitSwitch.get();
    }

}