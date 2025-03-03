package frc.robot.subsystems.endeffector;

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

    private boolean hasCoral = false;
    public EndEffectorSubsystem() {
        endeffectormotor1 = new SparkMax(Constants.EndEffectorConstants.endeffectormotor1ID, MotorType.kBrushless);
        SparkMaxConfig endeffectorConfig = new SparkMaxConfig();
        endeffectorConfig.idleMode(IdleMode.kBrake);
        endeffectorConfig.inverted(true);
        endeffectorConfig.smartCurrentLimit(25);
        endeffectormotor1.configure(endeffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

        // if (!hasCoral && (endeffectormotor1.getOutputCurrent() > 10 && endeffectormotor1.getAppliedOutput() > 0)) {
        //     has_coral_timer.start();
        // }

        // if (!hasCoral && has_coral_timer.isRunning() && !(endeffectormotor1.getOutputCurrent() > 10 && endeffectormotor1.getAppliedOutput() > 0)) {
        //     has_coral_timer.stop();
        //     has_coral_timer.reset();
        // }

        // if (hasCoral || has_coral_timer.hasElapsed(0.5)) {
        //     has_coral_timer.stop();
        //     has_coral_timer.reset();
        //     hasCoral = true;
        //     endeffectormotor1.set(0);
        // }
        
        SmartDashboard.putBoolean("Has Coral", hasCoral);
    }

    public void setEndEffector(double speed) {

        // if (hasCoral) {
        //     speed = 0;
        // }

        // if (hasCoral && speed < 0) {
        //     hasCoral = false;
        // }

        endeffectormotor1.set(speed);
    }

}
