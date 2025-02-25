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
    private final float ampThreshold = 2;
    private boolean noteBeThere = false;

    public EndEffectorSubsystem() {
        endeffectormotor1 = new SparkMax(Constants.EndEffectorConstants.endeffectormotor1ID, MotorType.kBrushless);
        SparkMaxConfig endeffectorConfig = new SparkMaxConfig();
        endeffectorConfig.idleMode(IdleMode.kBrake);
        endeffectormotor1.configure(endeffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setEndEffector (double speed) {
        endeffectormotor1.set(speed);
    }

    @Override
    public void periodic() {
        noteBeThere = (endeffectormotor1.getOutputCurrent() > ampThreshold);

        SmartDashboard.putNumber("Motor Current Current ", endeffectormotor1.getOutputCurrent());
        SmartDashboard.putBoolean("EndEffector Got Note ", noteBeThere);
    }
}