package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// one or two motors, 
public class EndEffectorSubsystem extends SubsystemBase {
    private final SparkMax endeffectormotor1;
    private final Timer timer;
    private boolean tempHasCoral = false;
    private boolean hasCoral = false;
    private boolean enabled = true;

    private enum State {
        HOLD,
        INTAKE,
        OUTTAKE,
        SCORE,
        DEALGIFY
    }

    private State currentState = State.HOLD;

    private final double senseCurrent = 14.0; // Current reached when a coral has been intook
    private final double senseDelay = 0.2; // Seconds after starting intaking to start sensing (to avoid motor start
                                           // spike)
    private final double extraIntakeTimer = 0.2; // Seconds to continue intaking after detecting coral

    private double senseTime = 0.0;

    public EndEffectorSubsystem() {
        timer = new Timer();
        endeffectormotor1 = new SparkMax(Constants.EndEffectorConstants.endeffectormotor1ID, MotorType.kBrushless);
        SparkMaxConfig endeffectorConfig = new SparkMaxConfig();
        endeffectorConfig.idleMode(IdleMode.kBrake);
        endeffectorConfig.inverted(true);
        endeffectorConfig.smartCurrentLimit(25);
        endeffectormotor1.configure(endeffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {

        // Set hasCoral if we are intaking, we are past the startup period, and we have
        // more current than normal
        if (currentState == State.INTAKE && !tempHasCoral && !hasCoral && timer.hasElapsed(senseDelay)
                && endeffectormotor1.getOutputCurrent() > senseCurrent) {
            tempHasCoral = true;
            senseTime = timer.get();
        }

        if (tempHasCoral && timer.get() - senseTime > extraIntakeTimer) { // If we have coral for more than 0.2 seconds,
                                                                          // stop intaking
            if (endeffectormotor1.getOutputCurrent() < senseCurrent) {
                tempHasCoral = false;
            } else {
                hasCoral = true;
                setHold();
            }
        }

        switch (currentState) {
            case HOLD:
                endeffectormotor1.set(0);
                break;
            case INTAKE, SCORE:
                endeffectormotor1.set(0.3);
                break;
            case OUTTAKE:
                endeffectormotor1.set(-0.3);
                break;
            case DEALGIFY:
                endeffectormotor1.set(0.7);
                break;
        }

        SmartDashboard.putString("Effector State", currentState.name());
        SmartDashboard.putBoolean("Has Coral", hasCoral);
        SmartDashboard.putNumber("Effector Current", endeffectormotor1.getOutputCurrent());
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setHold() {
        currentState = State.HOLD;
        timer.stop();
        timer.reset();
    }

    public void setIntake() {
        // Don't intake if we already have coral
        if (hasCoral) {
            return;
        }

        currentState = State.INTAKE;
        timer.start();
    }

    public void setOuttake() {
        currentState = State.OUTTAKE;
        timer.stop();
        timer.reset();
        hasCoral = false; // Assume any coral is leaving
        tempHasCoral = false; // Reset tempHasCoral to avoid false positives
    }

    public void setScore() {
        currentState = State.SCORE;
        timer.stop();
        timer.reset();
        hasCoral = false; // Assume any coral is leaving
        tempHasCoral = false; // Reset tempHasCoral to avoid false positives
    }

    public void setDealgify() {
        currentState = State.DEALGIFY;
    }
}
