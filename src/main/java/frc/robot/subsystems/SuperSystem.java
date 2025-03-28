package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
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
        
        L1(2.0, 60, 0.75),
        L2(3.0, 60, 0.75),
        L3(11.5, 55, 0.75),
        L4(27.0, 50, 0.85);

        
        private double elevator_position;
        private double arm_angle;
        private double reef_distance;

        ScoringPositions(double elevator_position, double arm_angle, double reef_distance) {
            this.elevator_position = elevator_position;
            this.arm_angle = arm_angle;
            this.reef_distance = reef_distance;
        }

        public double getElevatorPosition() {
            return elevator_position;
        }

        public double getArmAngle() {
            return arm_angle;
        }

        public double getReefDistance() {
            return reef_distance;
        }
    }

    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;

    private SuperStates state = SuperStates.INTAKE_CORAL;
    private ScoringPositions scoringPosition = ScoringPositions.L4;
    private boolean scoringLeft = true;

    private double dialPosition = 0;

    private double dialElevator = 0.0;
    private double dialArm = 0.0;
    private boolean dialControlArm = false;
    private boolean dialIsReady = false;

    public SuperSystem(ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    public void setDialPosition(double dialPosition) {
        this.dialPosition = dialPosition;
    }

    public void setDialContrlArm(boolean dialControlArm) {
        if (this.dialControlArm == dialControlArm) {
            return;
        }

        this.dialControlArm = dialControlArm;
        this.dialIsReady = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Scoring Position", this.scoringPosition.name() + " - " + (this.scoringLeft ? " Left" : "Right"));
        SmartDashboard.putString("Super State", this.state.name());

        if (DriverStation.isTest()) {

            if (dialControlArm && Math.abs(dialPosition - dialArm) < 0.05) {
                dialIsReady = true;
            }

            if (!dialControlArm && Math.abs(dialPosition - dialElevator) < 0.05) {
                dialIsReady = true;
            }

            if (dialIsReady) {
                if (dialControlArm) {
                    dialArm = dialPosition;
                    // dialArm = 90.0 - (dialPosition * 90.0);
                } else {
                    dialElevator = dialPosition;
                    // dialElevator = dialPosition * 27.5 + 0.5;
                }
            }
            
            elevator.setTargetPosition(dialElevator * 27.5 + 0.5);
            arm.setArmTargetPosition(90.0 - (dialArm * 90.0));
            return;
        }

        if (state == SuperStates.INTAKE_CORAL) {
            // Get the arm back in first, then move the elevator down
            if (elevator.getElevatorAngle() > 1.0) {
                arm.setTo90();
            } else {
                arm.setToIntake();
            }

            // Safe - wait for the arm to be in position before moving the elevator
            // if (arm.atIntake() || arm.at90()) {
            //     elevator.setTargetPosition(0.05);
            // }

            // YOLO - just go straight down without worrying about getting the arm in position.
            // we don't have time to wait, we only have time to win
            elevator.setTargetPosition(0.05);
        } else if (state == SuperStates.SCORE_CORAL || state == SuperStates.ELEVATOR_ONLY) {
            // if the elevator is at the wrong position, bring the arm in first
            if (!arm.safeToRaiseElevator() && !elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setSafeRaising();
            }

            // If the arm is upright, then we can move the elevator
            else if (arm.safeToRaiseElevator()) {
                elevator.setTargetPosition(scoringPosition.getElevatorPosition());
            }

            // Finally, if the elevator is at the right spot, we can score
            if (state != SuperStates.ELEVATOR_ONLY && elevator.atPosition(scoringPosition.getElevatorPosition())) {
                arm.setArmTargetPosition(scoringPosition.getArmAngle());
            }
        }  else if (state == SuperStates.CLIMB) {
            arm.setArmTargetPosition(60);
            elevator.setTargetPosition(0.05);
        } 
    }

    public void setScoringPosition(ScoringPositions position) {
        scoringPosition = position;
    }

    public ScoringPositions getScoringPosition() {
        return scoringPosition;
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
