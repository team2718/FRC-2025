package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SuperSystem extends SubsystemBase {

    private enum SuperStates {
        INTAKE_CORAL,
        SCORE_CORAL
    }

    public enum ScoringPositions {
        L1(10.0, 45.0),
        L2(15.0, 60.0),
        L3(20.0, 70.0),
        L4(25.0, 80.0);

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

    private ScoringPositions scoringPosition = ScoringPositions.L3;

    public SuperSystem(ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void periodic() {
        if (state == SuperStates.INTAKE_CORAL) {
            // Get the arm back in first, then move the elevator down
            arm.setTo90();

            if (arm.at90()) {
                elevator.setTargetPosition(0.5);
            }
        } else if (state == SuperStates.SCORE_CORAL) {
            // First, if the elevator is at the wrong position, bring the arm in first
            if (!arm.at90() && !elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setTo90();
            }

            // If the arm is upright, then we can move the elevator
            else if (arm.at90() && !elevator.atPosition(scoringPosition.getElevatorPosition())) {
                elevator.setTargetPosition(scoringPosition.getElevatorPosition());
            }

            // Finally, if the elevator is at the right spot, we can score
            else if (elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setArmTargetPosition(scoringPosition.getArmAngle());
            }
        }
    }

    public void setIntake() {
        state = SuperStates.INTAKE_CORAL;
    }

    public void setScoringPosition(ScoringPositions position) {
        scoringPosition = position;
    }

    public void setScoring() {
        state = SuperStates.SCORE_CORAL;
    }
}
