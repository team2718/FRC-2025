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
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final SparkAbsoluteEncoder armAbsoluteEncoder;
    

public ArmSubsystem() {
    armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.idleMode(IdleMode.kBrake);

    armConfig.inverted(true);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armFeedforward = new ArmFeedforward(0.25, 0.05, 0.08);
    armVoltagePID = new ProfiledPIDController(0.15, 0, 0.00095,
        new TrapezoidProfile.Constraints(60, 150), 0.02);

    armVoltagePID.setGoal(90);

    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();


}



public void setArm (double speed) {
    armMotor.set(speed);
    
}

public void stopArm() {
    armMotor.set(0);
}

public double getArmAngle() {
    return armAbsoluteEncoder.getPosition() * 360 + 21;
}

@Override
public void periodic() {
    updateArmLoop();
    SmartDashboard.putNumber("arm pos", getArmAngle());
}

public void setArmTargetPosition(double position) {
    armVoltagePID.setGoal(position);
}

public boolean atPosition() {
    return armVoltagePID.atGoal();
}

public boolean atPosition(double angle) {
    return Math.abs(getArmAngle() - angle) < armVoltagePID.getPositionTolerance();
}

public void setTo90() {
    setArmTargetPosition(90);
}

public boolean at90() {
    return atPosition(90);
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




