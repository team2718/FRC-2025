
package frc.robot.subsystems.elevator;

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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    private final ProfiledPIDController elevatorVoltagePID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final StatusSignal<Angle> elevatorRelativeEncoder;
    double startingposition = 0;
    public double currentVoltage;

    public double targetSetpointGoalThing = 0.5;

    private double targetSetpointPosition = 0.5;

    public ElevatorSubsystem() {
        talon_config.CurrentLimits.StatorCurrentLimit = 40;
        talon_config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        talon_config.Slot0.kG = 0.37;
        talon_config.Slot0.kS = 0.11;
        talon_config.Slot0.kV = 0.12;
        talon_config.Slot0.kA = 0.0008;
        talon_config.Slot0.kP = 0.9;
        talon_config.Slot0.kI = 0.0;
        talon_config.Slot0.kD = 0.0;
        talon_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        talon_config.MotionMagic.MotionMagicAcceleration = 60;
        talon_config.MotionMagic.MotionMagicCruiseVelocity = 30;

        elevatormotor1.getConfigurator().apply(talon_config);
        elevatormotor2.getConfigurator().apply(talon_config);

        // elevatormotor1.setControl(new MotionMagicVoltage));
        elevatormotor2.setControl(new Follower(Constants.ElevatorConstants.elevatormotor1ID, false));

        elevatorFeedforward = new ElevatorFeedforward(0.125, 0.355, 0.123, 0.0);
        elevatorVoltagePID = new ProfiledPIDController(0.0, 0, 0,
                new TrapezoidProfile.Constraints( 40, 30), 0.02);
        elevatorVoltagePID.setTolerance(Constants.ElevatorConstants.elevatorTolerance);
        elevatorRelativeEncoder = elevatormotor1.getRotorPosition();
    }


    @Override
    public void periodic() {
        // updateElevatorLoop();
        elevatormotor1.setControl(new MotionMagicVoltage(targetSetpointPosition));

        SmartDashboard.putNumber("Elevator Position", getElevatorAngle());
        SmartDashboard.putNumber("Elevator Velocity", elevatormotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Desired Elevator Position", targetSetpointPosition);
        SmartDashboard.putNumber("Desired Elevator Velocity", elevatorVoltagePID.getSetpoint().velocity);
    }

    public void resetPosition() {
        startingposition = elevatorRelativeEncoder.getValueAsDouble();
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
        targetSetpointPosition = position;
        SmartDashboard.putNumber("Elevator Voltage", elevatormotor1.getMotorVoltage().getValueAsDouble());
        // elevatorVoltagePID.setGoal(position);
    }

    public void resetProfilePID() {
        elevatorVoltagePID.reset(getElevatorAngle());
    }

    public void updateElevatorLoop() {
        double voltage = elevatorVoltagePID.calculate(getElevatorAngle()) + elevatorFeedforward
                .calculate(elevatorVoltagePID.getSetpoint().velocity);

        voltage = Math.max(-4, Math.min(5, voltage));

        // if (voltage < 0.4 && getElevatorAngle() < 0.7) {
        //     voltage = 0.6;
        // }

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