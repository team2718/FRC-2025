package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SuperSystem extends SubsystemBase {

    private enum SuperStates {
        INTAKE_CORAL,
        SCORE_CORAL,
        ELEVATOR_ONLY,
        CLIMB
    }

    public enum ScoringPositions {
        L1(3, 38.1),
        L2(8.2, 29),
        L3(16.5, 29),
        L4(29.0, 28);

        private double elevator_position;
        private double arm_angle;

        ScoringPositions(double elevator_position, double arm_angle) {
            this.elevator_position = elevator_position;
            this.arm_angle = arm_angle;
        }

        public double getElevatorPosition() {
            return elevator_position;
        }

        public double getArmAngle() {
            return arm_angle;
        }
    }

    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;

    private SuperStates state = SuperStates.INTAKE_CORAL;
    private ScoringPositions scoringPosition = ScoringPositions.L4;
    private boolean scoringLeft = true;

    public SuperSystem(ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Scoring Position", this.scoringPosition.name() + " - " + (this.scoringLeft ? " Left" : "Right"));
        SmartDashboard.putString("Super State", this.state.name());

        if (state == SuperStates.INTAKE_CORAL) {
            // Get the arm back in first, then move the elevator down
            arm.setTo90();

            if (arm.at90()) {
                elevator.setTargetPosition(0.6);
            }
        } else if (state == SuperStates.SCORE_CORAL || state == SuperStates.ELEVATOR_ONLY) {
            // if the elevator is at the wrong position, bring the arm in first
            if (!arm.atSafeRaising() && !elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setSafeRaising();
            }

            // If the arm is upright, then we can move the elevator
            else if (arm.atSafeRaising() && !elevator.atPosition(scoringPosition.getElevatorPosition())) {
                elevator.setTargetPosition(scoringPosition.getElevatorPosition());
            }

            // Finally, if the elevator is at the right spot, we can score
            else if (state != SuperStates.ELEVATOR_ONLY && elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setArmTargetPosition(scoringPosition.getArmAngle());
            }
        }  else if (state == SuperStates.CLIMB) {
            arm.setArmTargetPosition(32);
            elevator.setTargetPosition(2);
        } 
    }

    public void setScoringPosition(ScoringPositions position) {
        scoringPosition = position;
    }

    public void setIntake() {
        state = SuperStates.INTAKE_CORAL;
    }

    public void setScoring() {
        state = SuperStates.SCORE_CORAL;
    }

    public void setMoveAuto() {
        state = SuperStates.ELEVATOR_ONLY;
    }

    public boolean isScoringLeft() {
        return scoringLeft;
    }

    public void setScoringLeft() {
        this.scoringLeft = true;
    }

    public void setScoringRight() {
        this.scoringLeft = false;
    }

    public void setClimbState() {
        state = SuperStates.CLIMB;
    }
}
