package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GripperArmConstants;

public class SuperstructureSubsystem extends SubsystemBase {

    private final ElevatorSubsystem elevator;
    private final GripperArmSubsystem arm;

    private SuperstructureTarget currentTarget = SuperstructureTarget.HOME;

    public enum SuperstructureTarget {
        HOME,

        CORAL_PICKUP,

        L1_CORAL_SCORE,
        L2_CORAL_SCORE,
        L3_CORAL_SCORE,
        L4_CORAL_SCORE,

        L1_CORAL_LOWER,
        L2_CORAL_LOWER,
        L3_CORAL_LOWER,
        L4_CORAL_LOWER,

        ALGAE_HOME,
        ALGAE_FLOOR_PICKUP,
        ALGAE_L2_PICKUP,
        ALGAE_L3_PICKUP,

        NET_SCORE,
        PROCESSOR_SCORE
    }

    public SuperstructureSubsystem(
            ElevatorSubsystem elevator,
            GripperArmSubsystem arm
    ) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public void goHome() {
        currentTarget = SuperstructureTarget.HOME;
    }

    public void goToCoralPickup() {
        currentTarget = SuperstructureTarget.CORAL_PICKUP;
    }

    public void goToL1CoralScore() {
        currentTarget = SuperstructureTarget.L1_CORAL_SCORE;
    }

    public void goToL2CoralScore() {
        currentTarget = SuperstructureTarget.L2_CORAL_SCORE;
    }

    public void goToL3CoralScore() {
        currentTarget = SuperstructureTarget.L3_CORAL_SCORE;
    }

    public void goToL4CoralScore() {
        currentTarget = SuperstructureTarget.L4_CORAL_SCORE;
    }

    public void goToL1SplitScore() {
        currentTarget = SuperstructureTarget.L1_CORAL_LOWER;
    }

    public void goToL2CoralLower() {
        currentTarget = SuperstructureTarget.L2_CORAL_LOWER;
    }

    public void goToL3CoralLower() {
        currentTarget = SuperstructureTarget.L3_CORAL_LOWER;
    }

    public void goToL4CoralLower() {
        currentTarget = SuperstructureTarget.L4_CORAL_LOWER;
    }

    public void goToAlgaeHome() {
        currentTarget = SuperstructureTarget.ALGAE_HOME;
    }

    public void goToAlgaeFloorPickup() {
        currentTarget = SuperstructureTarget.ALGAE_FLOOR_PICKUP;
    }

    public void goToL2AlgaePickup() {
        currentTarget = SuperstructureTarget.ALGAE_L2_PICKUP;
    }

    public void goToL3AlgaePickup() {
        currentTarget = SuperstructureTarget.ALGAE_L3_PICKUP;
    }

    public void goToNetScore() {
        currentTarget = SuperstructureTarget.NET_SCORE;
    }

    public void goToProcessorScore() {
        currentTarget = SuperstructureTarget.PROCESSOR_SCORE;
    }

    public boolean atTarget() {
        return elevator.atTarget() && arm.atTarget();
    }
  
    @Override
    public void periodic() {

        switch (currentTarget) {

            case HOME -> {
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
            }

            case CORAL_PICKUP -> {
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.INTAKE);
            }

            case L1_CORAL_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.L1,
                    GripperArmSubsystem.ArmPosition.SCORE_L1
            );

            case L2_CORAL_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.L2,
                    GripperArmSubsystem.ArmPosition.SCORE_L2
            );

            case L3_CORAL_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.L3,
                    GripperArmSubsystem.ArmPosition.SCORE_L3
            );

            case L4_CORAL_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.L4,
                    GripperArmSubsystem.ArmPosition.SCORE_L4
            );

            case L1_CORAL_LOWER,
                 L2_CORAL_LOWER,
                 L3_CORAL_LOWER,
                 L4_CORAL_LOWER -> {
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
            }

            case ALGAE_HOME -> {
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
            }

            case ALGAE_FLOOR_PICKUP -> {
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.INTAKE);
            }

            case ALGAE_L2_PICKUP -> {
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.L2);
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.INTAKE);
            }

            case ALGAE_L3_PICKUP -> {
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.L3);
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.INTAKE);
            }

            case NET_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.NET,
                    GripperArmSubsystem.ArmPosition.SCORE_NET
            );

            case PROCESSOR_SCORE -> goToScore(
                    ElevatorSubsystem.ElevatorPosition.PROCESSOR,
                    GripperArmSubsystem.ArmPosition.SCORE_NET
            );
            default -> {
                arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
                elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
            }
            
        }
    }


    private void goToScore(
            ElevatorSubsystem.ElevatorPosition elevatorPos,
            GripperArmSubsystem.ArmPosition armPos
    ) {

        if (arm.getArmAngleDegrees() < SuperstructureConstants.SAFE_ARM_ANGLE_DEG) {
            arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
            elevator.setPosition(ElevatorSubsystem.ElevatorPosition.HOME);
            return;
        }        

        elevator.setPosition(elevatorPos);
        arm.setArmPosition(armPos);
    }

    public SuperstructureTarget getCurrentTarget() {
        return currentTarget;
    }
    public void stop() {
    arm.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
    elevator.stop();
    }

}
