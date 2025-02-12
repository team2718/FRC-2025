
package frc.robot.subsystems.elevator;

import 
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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;

// 2 motors, one is follower, PID, feedforward, hall effect sensor, use relative encoder on lead motor



public class ElevatorSubsystem extends SubsystemBase {
    final TalonFX elevatormotor1 = new TalonFX(Constants.ElevatorConstants.elevatormotor1ID);
    final TalonFX elevatormotor2 = new TalonFX(Constants.ElevatorConstants.elevatormotor2ID);
    private TalonFXConfiguration talon_config = new TalonFXConfiguration();
    final VoltageOut voltageControl = new VoltageOut(0.0);

public ElevatorSubsystem() {
    talon_config.CurrentLimits.StatorCurrentLimit = 40;
    talon_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatormotor1.getConfigurator().apply(talon_config);
    elevatormotor2.getConfigurator().apply(talon_config);

    elevatormotor1.setControl(voltageControl.withOutput(0.0));
    elevatormotor2.setControl(voltageControl.withOutput(0.0));
    
}
@Override
    public void periodic() {
        
    }

}