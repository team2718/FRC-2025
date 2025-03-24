
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

// 2 motors, one is follower, PID, feedforward, hall effect sensor, use relative encoder on lead motor

public class ElevatorSubsystem extends SubsystemBase {
    final TalonFX elevatormotor1 = new TalonFX(Constants.ElevatorConstants.elevatormotor1ID);
    final TalonFX elevatormotor2 = new TalonFX(Constants.ElevatorConstants.elevatormotor2ID);
    private TalonFXConfiguration talon_config = new TalonFXConfiguration();
    final VoltageOut voltageControl = new VoltageOut(0.0);

    public double targetSetpointGoalThing = 0.5;
    private double targetSetpointPosition = 0.5;

    private boolean enabled = true; // used to enable/disable elevator control

    Alert elevatorMotor1Alert;
    Alert elevatorMotor2Alert;

    public ElevatorSubsystem() {
        talon_config.CurrentLimits.StatorCurrentLimit = 40;
        talon_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talon_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talon_config.Slot0.kG = 0.5; // TODO: retune after post-MOSE redesign
        talon_config.Slot0.kS = 0.11; // TODO: retune after post-MOSE redesign
        talon_config.Slot0.kV = 0.131;
        talon_config.Slot0.kA = 0.0011;
        talon_config.Slot0.kP = 1.8;
        talon_config.Slot0.kI = 0.0;
        talon_config.Slot0.kD = 0.0;
        talon_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        talon_config.MotionMagic.MotionMagicCruiseVelocity = 50;
        talon_config.MotionMagic.MotionMagicAcceleration = 75;
        talon_config.MotionMagic.MotionMagicJerk = 2000;

        elevatormotor1.getConfigurator().apply(talon_config);
        elevatormotor2.getConfigurator().apply(talon_config);

        elevatormotor1.getPosition().setUpdateFrequency(100);
        elevatormotor1.setPosition(0.0, 1.0); // reset position to 0

        // elevatormotor1.setControl(new MotionMagicVoltage));
        elevatormotor2.setControl(new Follower(Constants.ElevatorConstants.elevatormotor1ID, false));

        elevatorMotor1Alert = new Alert("Motor \"" + "elevatorMotor1" + "\" is not connected!", AlertType.kError);
        elevatorMotor2Alert = new Alert("Motor \"" + "elevatorMotor2" + "\" is not connected!", AlertType.kError);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        if (enabled) {
            elevatormotor1.setControl(new MotionMagicVoltage(targetSetpointPosition));
        } else {
            elevatormotor1.setControl(new NeutralOut());
        }

        SmartDashboard.putNumber("Elevator Position", getElevatorAngle());
        SmartDashboard.putNumber("Elevator Velocity", elevatormotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Desired Elevator Position", targetSetpointPosition);
        SmartDashboard.putNumber("Elevator Voltage", elevatormotor1.getMotorVoltage().getValueAsDouble());
    }

    public boolean atPosition() {
        return Math.abs(
                getElevatorAngle() - targetSetpointPosition) < Constants.ElevatorConstants.elevatorPositionTolerance;
    }

    public boolean atPosition(double position) {
        return Math.abs(getElevatorAngle() - position) < Constants.ElevatorConstants.elevatorPositionTolerance;
    }

    public void setTargetPosition(double position) {
        targetSetpointPosition = position;
    }

    public double getElevatorAngle() {
        return elevatormotor1.getPosition().getValueAsDouble();

    }

    public void setVoltage(Voltage volts) {
        elevatormotor1.setVoltage(volts.in(Volts));
        elevatormotor2.setVoltage(volts.in(Volts));
    }

    // alerts us if the 1st elevator motor is not detected
    public Alert setElevatorMotor1Alert() {
        if (elevatormotor1.isConnected()) {
            elevatorMotor1Alert.set(false); 
        } else {
            elevatorMotor1Alert.set(true);
        }
        
        return elevatorMotor1Alert;
    }
    
    // alerts us if the 2nd elevator motor is not detected
    public Alert setElevatorMotor2Alert() {
        if (elevatormotor2.isConnected()) {
            elevatorMotor2Alert.set(false); 
        } else {
            elevatorMotor2Alert.set(true);
        }
        
        return elevatorMotor2Alert;
    }

}