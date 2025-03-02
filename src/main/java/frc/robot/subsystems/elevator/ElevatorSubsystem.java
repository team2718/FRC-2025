
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.VoltageUnit;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

// 2 motors, one is follower, PID, feedforward, hall effect sensor, use relative encoder on lead motor

public class ElevatorSubsystem extends SubsystemBase {
    final TalonFX elevatormotor1 = new TalonFX(Constants.ElevatorConstants.elevatormotor1ID);
    final TalonFX elevatormotor2 = new TalonFX(Constants.ElevatorConstants.elevatormotor2ID);
    private TalonFXConfiguration talon_config = new TalonFXConfiguration();
    final VoltageOut voltageControl = new VoltageOut(0.0);
    private final ProfiledPIDController elevatorVoltagePID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final StatusSignal<Angle> elevatorRelativeEncoder;
    double startingposition = 0;
    public double currentVoltage;

    public double targetSetpointGoalThing = 0.5;

    public ElevatorSubsystem() {
        talon_config.CurrentLimits.StatorCurrentLimit = 40;
        talon_config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        elevatormotor1.getConfigurator().apply(talon_config);
        elevatormotor2.getConfigurator().apply(talon_config);

        elevatormotor1.setControl(voltageControl.withOutput(0.0));
        elevatormotor2.setControl(voltageControl.withOutput(0.0));

        talon_config.CurrentLimits.StatorCurrentLimit = 40;
        talon_config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        

        elevatorFeedforward = new ElevatorFeedforward(0.12, 0.36, 0.11, 0.002);
        elevatorVoltagePID = new ProfiledPIDController(0.03, 0, 0,
                new TrapezoidProfile.Constraints( 70, 70), 0.02);
        elevatorVoltagePID.setTolerance(Constants.ElevatorConstants.elevatorTolerance);
        elevatorRelativeEncoder = elevatormotor1.getRotorPosition();

        startingposition = elevatorRelativeEncoder.getValueAsDouble();

    }


    @Override
    public void periodic() {
        updateElevatorLoop();

        SmartDashboard.putNumber("Elevator Position", getElevatorAngle());
        SmartDashboard.putNumber("Elevator Velocity", elevatormotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Desired Elevator Position", elevatorVoltagePID.getSetpoint().position);
        SmartDashboard.putNumber("Desired Elevator Velocity", elevatorVoltagePID.getSetpoint().velocity);
    }

    public boolean atPosition() {
        return elevatorVoltagePID.atGoal();
    }

    public boolean atPosition(double position) {
        return Math.abs(getElevatorAngle() - position) < elevatorVoltagePID.getPositionTolerance(); 
    }

    public void stopElevator() {
        elevatormotor1.set(0);
    }

    public void incrementGoal() {
        targetSetpointGoalThing += 0.5;
        elevatorVoltagePID.setGoal(targetSetpointGoalThing);
    }

    public void decrementGoal() {
        targetSetpointGoalThing -= 0.5;
        elevatorVoltagePID.setGoal(targetSetpointGoalThing);

    }

    public void elevatorGo(double voltage) {
        elevatormotor1.set(voltage);
    }

    public void setTargetPosition(double position) {
        elevatorVoltagePID.setGoal(position);
    }

    public void resetProfilePID() {
        elevatorVoltagePID.reset(getElevatorAngle());
    }

    public void updateElevatorLoop() {
        double voltage = elevatorVoltagePID.calculate(getElevatorAngle()) + elevatorFeedforward
                .calculate(elevatorVoltagePID.getSetpoint().velocity);

        // horrible disgusting bad hack
        // if (getElevatorAngle() < 1.5 && voltage > 0) {
        //     voltage = voltage * 1.5;
        // }

        voltage = Math.max(-7, Math.min(7, voltage));

        setVoltage(Volts.of(voltage));

        SmartDashboard.putNumber("Elevator Voltage", voltage);
    }

    public double getElevatorAngle() {
        return elevatormotor1.getRotorPosition().getValueAsDouble() - startingposition;

    }

    public void setVoltage(Voltage volts) {

        elevatormotor1.setVoltage(volts.in(Volts));
        elevatormotor2.setVoltage(volts.in(Volts));
    }

}