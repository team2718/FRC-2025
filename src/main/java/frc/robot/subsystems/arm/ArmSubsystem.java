package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkClosedLoopController ArmPIDController;
    

public ArmSubsystem() {
    armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.idleMode(IdleMode.kBrake);

    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(0.0, 1.0)
        .pidf(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ArmPIDController = armMotor.getClosedLoopController();


}

public static class PIDF {
    /* Feedforward constant for PID loop */
    public static final double FEEDFORWARD = 0.000199;
    /* Porportion constant for PID loop */
    public static final double PORPORTION = 0.001;
    /* Integral constant for PID loop */
    public static final double INTEGRAL = 0;
    /* Derivative constant for PID loop */
    public static final double DERIVATIVE = 0.0;
  }

public void setSpeed(double power) {
    armMotor.set(power);
}

public void setArm (double speed) {
    armMotor.set(speed);
    
}

// public void setArmMotor (double speed) {
    // ArmPIDController.setReference(speed);
// }
}
