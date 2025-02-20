package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final ProfiledPIDController armVoltagePID;
    private final ArmFeedforward armFeedforward;
    private final RelativeEncoder armRelativeEncoder;
    

public ArmSubsystem() {
    armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.idleMode(IdleMode.kBrake);

   

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armFeedforward = new ArmFeedforward(0, 0, 0);
    armVoltagePID = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(0, 0), 0.02);

    armRelativeEncoder = armMotor.getEncoder();


}



public void setArm (double speed) {
    armMotor.set(speed);
    
}

public void stopArm() {
    armMotor.set(0);
}

public double getArmAngle() {
    return armRelativeEncoder.getPosition();
}

@Override
public void periodic() {
}

public void updateArmLoop() {
    double voltage = armVoltagePID.calculate(getArmAngle()) +  armFeedforward.calculate(armVoltagePID.getSetpoint().position, armVoltagePID.getSetpoint().velocity);
    voltage = Math.max(-7, Math.min(7, voltage));

    setVoltage(Volts.of(voltage));

}

public void setVoltage(Voltage volts) {
    
        armMotor.setVoltage(volts);
    }
}




