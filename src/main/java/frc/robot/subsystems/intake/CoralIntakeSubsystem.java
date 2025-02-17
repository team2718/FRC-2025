package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase{

    private final SparkMax Intakemotor;
    private final SparkMax indexerMotor;

    private enum INTAKE_STATE {
        INTAKE, OUTTAKE, HOLD
    };

    private INTAKE_STATE intakeState = INTAKE_STATE.HOLD;

    public CoralIntakeSubsystem() {
        Intakemotor = new SparkMax(Constants.IntakeConstants.IntakemotorID, SparkLowLevel.MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.IntakeConstants.indexerMotorID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig IntakeConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();

        Intakemotor.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        // sets intake speed
    public void setSpeed(double power) {
        Intakemotor.set(power);
    }

    // stops intake
    public void stopIntake() {
        Intakemotor.set(0);
    }

    // for backwards intake
    public void setbackSpeed(double power) {
        Intakemotor.set(-power);
    }

    // for backwards index
    public void setIndexerback(double power) {
        indexerMotor.set(-power);
    }

    // sets indexer motor speed
    public void indexerSpeed(double power) {
        indexerMotor.set(power);
    }

    // stops indexer
    public void stopIndexer() {
        indexerMotor.set(0);
    }

    public double getSpeed() {
        return Intakemotor.get();
    }

    public Command runIntake(double Speed) {
        return run(() -> {
            setSpeed(Speed);
        });
    }

    @Override
    public void periodic() {
  
    }

    public void setStateIntake() {
        intakeState = INTAKE_STATE.INTAKE;
    }

    public void setStateOuttake() {
        intakeState = INTAKE_STATE.OUTTAKE;
    }

    public void setStateHold() {
        intakeState = INTAKE_STATE.HOLD;
    }

    public void stopIntakePeriodic() {
        if (robotHasCoral() && intakeState == INTAKE_STATE.INTAKE) {
            intakeState = INTAKE_STATE.HOLD;
            Intakemotor.set(0);
            indexerMotor.set(0);
        }

        switch (intakeState) {
            case INTAKE:
            Intakemotor.set(0.7);
                indexerMotor.set(0.7);
                break;
            case OUTTAKE:
            Intakemotor.set(-0.5);
                indexerMotor.set(-0.5);
                break;
            default:
                Intakemotor.set(0);
                indexerMotor.set(0);
                break;
        }
    }

    // sets intake in commmand
    public void setIntake(double speed) {
        Intakemotor.set(speed);
        indexerMotor.set(speed);
    }

    // testing if the robot has the coral, probably for LEDs (placeholder for now)
    public boolean robotHasCoral() {
        //last year: "return !IntakeLimitSwitch.get()""
        return true;
    }


}
