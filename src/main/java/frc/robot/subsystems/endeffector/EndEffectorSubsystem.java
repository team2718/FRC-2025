package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

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

    private final LaserCan lc = new LaserCan(47);

    private enum State {
        HOLD,
        INTAKE,
        OUTTAKE,
        SCORE,
        DEALGIFY
    }

    private State currentState = State.HOLD;

    private final double senseCurrent = 19.0; // Current reached when a coral has been intook
    private final double senseDelay = 0.2; // Seconds after starting intaking to start sensing (to avoid motor start
                                           // spike)
    private final double extraIntakeTimer = 0.2; // Seconds to continue intaking after detecting coral

    private double senseTime = 0.0;

    Alert endEffectorAlert;

    public EndEffectorSubsystem() {
        timer = new Timer();
        endeffectormotor1 = new SparkMax(Constants.EndEffectorConstants.endeffectormotor1ID, MotorType.kBrushless);
        SparkMaxConfig endeffectorConfig = new SparkMaxConfig();
        endeffectorConfig.idleMode(IdleMode.kBrake);
        endeffectorConfig.inverted(true);
        endeffectorConfig.smartCurrentLimit(25);
        endeffectormotor1.configure(endeffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        endEffectorAlert = new Alert("Motor \"" + "End Effector" + "\" is not connected!", AlertType.kError);

        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {

        if (!enabled) {
            endeffectormotor1.set(0);
            return;
        }

        Measurement laser_measurement = lc.getMeasurement();
        if (laser_measurement != null) {
            hasCoral = laser_measurement.distance_mm < 10;
        } else {
            hasCoral = false;
        }

        if (currentState == State.INTAKE && hasCoral) {
            currentState = State.HOLD;
        }

        switch (currentState) {
            case HOLD:
                endeffectormotor1.set(0);
                break;
            case INTAKE, SCORE:
                endeffectormotor1.set(0.4);
                break;
            case OUTTAKE:
                endeffectormotor1.set(-0.5);
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

    // alerts us if the end effector motor is not detected
    public Alert setEndEffectorAlert() {
        if (endeffectormotor1.getBusVoltage() > 0) {
            endEffectorAlert.set(false);
        } else {
            endEffectorAlert.set(true);
        }

        return endEffectorAlert;
    }

}
