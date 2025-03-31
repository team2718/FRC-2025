package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    Alert armMotorAlert;

    public final double intakePosition = 91.0; // intaking angle
    public final double position90 = 85.0; // upright angle
    public final double safeRaisingPosition = 75.0; // safe raising angle

    private boolean enabled = true; // used to enable/disable arm control

    public ArmSubsystem() {
        armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.idleMode(IdleMode.kBrake);
        armConfig.smartCurrentLimit(20);
        armConfig.inverted(true);

        armConfig.absoluteEncoder.zeroCentered(true);
        armConfig.absoluteEncoder.zeroOffset(0.925);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armFeedforward = new ArmFeedforward(0.14, 0.24, 0.14);
        armVoltagePID = new ProfiledPIDController(0.15, 0, 0,
                new TrapezoidProfile.Constraints(150, 200), 0.02);
        armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

        armMotorAlert = new Alert("Motor \"" + "Arm Motor" + "\" is faulting!", AlertType.kError);

        armVoltagePID.setGoal(90);
        armVoltagePID.setTolerance(Constants.ArmConstants.armPositionTolerance);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getArmAngle() {
        double degrees = (armAbsoluteEncoder.getPosition()) * 360; // TODO: Remove the 0.67 offset after fixing
                                                                   // the encoder zeroing
        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    @Override
    public void periodic() {
        setAlerts();

        if (enabled) {
            updateArmLoop();
        } else {
            setVoltage(Volts.of(0));
        }

        SmartDashboard.putNumber("Arm Position", getArmAngle());
        SmartDashboard.putNumber("Arm Velocity", armAbsoluteEncoder.getVelocity() * 375 / 60);
        SmartDashboard.putNumber("Desired Arm Position", armVoltagePID.getSetpoint().position);
        SmartDashboard.putNumber("Desired Arm Velocity", armVoltagePID.getSetpoint().velocity);

    }

    public void setArmTargetPosition(double position) {
        armVoltagePID.setGoal(position);
    }

    public double getArmTargetPosition() {
        return armVoltagePID.getGoal().position;
    }

    public boolean atPosition() {
        return armVoltagePID.atGoal();
    }

    public boolean atPosition(double angle) {
        return Math.abs(getArmAngle() - angle) < armVoltagePID.getPositionTolerance();
    }

    public void setToIntake() {
        setArmTargetPosition(intakePosition);
    }

    public void setTo90() {
        setArmTargetPosition(position90);
    }

    public void setSafeRaising() {
        setArmTargetPosition(safeRaisingPosition);
    }

    public boolean atIntake() {
        return atPosition(intakePosition);
    }

    public boolean at90() {
        return atPosition(position90);
    }

    public boolean safeToRaiseElevator() {
        return getArmAngle() > (safeRaisingPosition - Constants.ArmConstants.armPositionTolerance);
    }

    public void resetProfilePID() {
        armVoltagePID.reset(getArmAngle());
    }

    public void updateArmLoop() {
        double voltage = armVoltagePID.calculate(getArmAngle())
                + armFeedforward.calculate(armVoltagePID.getSetpoint().position, armVoltagePID.getSetpoint().velocity);
        voltage = Math.max(-7, Math.min(7, voltage));

        setVoltage(Volts.of(voltage));

    }

    public void setVoltage(Voltage volts) {

        armMotor.setVoltage(volts);
    }

    // alerts us if the arm motor is not detected
    public void setAlerts() {
        armMotorAlert.set(armMotor.hasActiveFault());
    }
}
