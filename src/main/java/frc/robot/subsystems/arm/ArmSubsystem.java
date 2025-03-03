package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final ProfiledPIDController armVoltagePID;
    private final ArmFeedforward armFeedforward;
    private final SparkAbsoluteEncoder armAbsoluteEncoder;
    

public ArmSubsystem() {
    armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();
    absoluteEncoderConfig.zeroCentered(true);
    absoluteEncoderConfig.zeroOffset(0.0);

    armConfig.idleMode(IdleMode.kBrake);
    armConfig.smartCurrentLimit(20);
    armConfig.inverted(true);
    armConfig.absoluteEncoder.apply(absoluteEncoderConfig);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armFeedforward = new ArmFeedforward(0.165, 0.055, 0.13);
    armVoltagePID = new ProfiledPIDController(0.15, 0, 0,
        new TrapezoidProfile.Constraints(100, 100), 0.02);

    armVoltagePID.setGoal(90);
    armVoltagePID.setTolerance(3.5);


    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
    
}



public void setArm (double speed) {
    armMotor.set(speed);
    
}

public void stopArm() {
    armMotor.set(0);
}

public double getArmAngle() {
    double degrees = (armAbsoluteEncoder.getPosition() - 0.67) * 360;
    if (degrees < 0) {
        degrees += 360;
    }

    return degrees;
}

@Override
public void periodic() {
     updateArmLoop();


    SmartDashboard.putNumber("Arm Position", getArmAngle());
    SmartDashboard.putNumber("Arm Velocity", armAbsoluteEncoder.getVelocity() * 375 / 60);
    SmartDashboard.putNumber("Desired Arm Position", armVoltagePID.getSetpoint().position);
    SmartDashboard.putNumber("Desired Arm Velocity", armVoltagePID.getSetpoint().velocity);

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
    setArmTargetPosition(85);
}

public void setSafeRaising() {
    setArmTargetPosition(80);
}

public boolean at90() {
    return atPosition(85);
}

public boolean atSafeRaising() {
    return atPosition(80);
}

public void resetProfilePID() {
    armVoltagePID.reset(getArmAngle());
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




