package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase{

    private final SparkMax topIntakemotor;
    private final SparkMax bottomIntakemotor;
    private final SparkMax indexerMotor;

    private enum INTAKE_STATE {
        INTAKE, OUTTAKE, HOLD
    };

    private INTAKE_STATE intakeState = INTAKE_STATE.HOLD;

    public CoralIntakeSubsystem() {
        topIntakemotor = new SparkMax(Constants.IntakeConstants.topIntakemotorID, SparkLowLevel.MotorType.kBrushless);
        bottomIntakemotor = new SparkMax(Constants.IntakeConstants.bottomIntakemotorID, SparkLowLevel.MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.IntakeConstants.indexerMotorID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig topIntakeConfig = new SparkMaxConfig();
        SparkMaxConfig bottomIntakeConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();

        topIntakemotor.configure(topIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomIntakemotor.configure(bottomIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        // sets intake speed
    public void setSpeed(double power) {
        topIntakemotor.set(power);
        bottomIntakemotor.set(power);
    }

    // stops intake
    public void stopIntake() {
        topIntakemotor.set(0);
        bottomIntakemotor.set(0);
    }

    // for backwards intake
    public void setbackSpeed(double power) {
        topIntakemotor.set(-power);
        bottomIntakemotor.set(-power);
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
        return topIntakemotor.get();
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
        if (hasCoral() && intakeState == INTAKE_STATE.INTAKE) {
            intakeState = INTAKE_STATE.HOLD;
            topIntakemotor.set(0);
            bottomIntakemotor.set(0);
            indexerMotor.set(0);
        }

        switch (intakeState) {
            case INTAKE:
            topIntakemotor.set(0.7);
            bottomIntakemotor.set(0.7);
                indexerMotor.set(0.7);
                break;
            case OUTTAKE:
            topIntakemotor.set(-0.5);
            bottomIntakemotor.set(-0.5);
                indexerMotor.set(-0.5);
                break;
            default:
                topIntakemotor.set(0);
                bottomIntakemotor.set(0);
                indexerMotor.set(0);
                break;
        }
    }

    // sets intake in commmand
    public void setIntake(double speed) {
        topIntakemotor.set(speed);
        bottomIntakemotor.set(speed);
        indexerMotor.set(speed);
    }

    // testing if the robot has the coral (placeholder for now)
    public boolean hasCoral() {
        return true;
    }


}