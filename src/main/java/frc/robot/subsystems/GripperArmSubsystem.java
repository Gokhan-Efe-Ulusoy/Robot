package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.GripperArmConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GripperArmSubsystem extends SubsystemBase{

    private final TalonFX armMotor;
    private final TalonFX gripperMotor;

    private final DigitalInput beamBreak;

    private double targetAngleDeg = 0.0;

    public enum ArmPosition {
        STOW,
        INTAKE,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        SCORE_NET
    }

    public GripperArmSubsystem() {
        armMotor = new TalonFX(GripperArmConstants.ARM_MOTOR_ID);
        gripperMotor = new TalonFX(GripperArmConstants.GRIPPER_MOTOR_ID);

        beamBreak = new DigitalInput(GripperArmConstants.BEAM_BREAK_DIO);

        configureArmMotor();
        configureGripperMotor();

        armMotor.setPosition(0);
    }

    private void configureArmMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentThreshold = 45.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.2;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(GripperArmConstants.MAX_ANGLE_DEG);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(GripperArmConstants.MIN_ANGLE_DEG);

        config.MotionMagic.MotionMagicCruiseVelocity =
                GripperArmConstants.ARM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration =
                GripperArmConstants.ARM_ACCELERATION;

        config.Slot0.kP = GripperArmConstants.kP;
        config.Slot0.kD = GripperArmConstants.kD;

        armMotor.getConfigurator().apply(config);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureGripperMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
    
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentThreshold = 30.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
    
        gripperMotor.getConfigurator().apply(config);
        gripperMotor.setNeutralMode(NeutralModeValue.Coast);
    }

  
    public void setArmPosition(ArmPosition position) {
        switch (position) {
            case STOW -> setArmAngleDegrees(GripperArmConstants.STOW_ANGLE);
            case INTAKE -> setArmAngleDegrees(GripperArmConstants.INTAKE_ANGLE);
            case SCORE_L1 -> setArmAngleDegrees(GripperArmConstants.SCORE_L1_ANGLE);
            case SCORE_L2 -> setArmAngleDegrees(GripperArmConstants.SCORE_L2_ANGLE);
            case SCORE_L3 -> setArmAngleDegrees(GripperArmConstants.SCORE_L3_ANGLE);
            case SCORE_L4 -> setArmAngleDegrees(GripperArmConstants.SCORE_L4_ANGLE);
            case SCORE_NET -> setArmAngleDegrees(GripperArmConstants.SCORE_NET_ANGLE);
        }
    }
    public void setArmAngleDegrees(double degrees) {
        degrees = MathUtil.clamp(
                degrees,
                GripperArmConstants.MIN_ANGLE_DEG,
                GripperArmConstants.MAX_ANGLE_DEG
        );

        targetAngleDeg = degrees;

        double rotations = degreesToMotorRotations(degrees);
        armMotor.setControl(new MotionMagicVoltage(rotations));
    }
    public boolean atTarget() {
        return Math.abs(getArmAngleDegrees() - targetAngleDeg) < 1.0;
    }

    public double getArmAngleDegrees() {
        return motorRotationsToDegrees(
                armMotor.getPosition().getValue()
        );
    }

  
    public void intake() {
        gripperMotor.set(GripperArmConstants.INTAKE_SPEED);
    }
    public void hold() {
        gripperMotor.set(GripperArmConstants.HOLD_SPEED);
    }

    public void outtake() {
        gripperMotor.set(GripperArmConstants.OUTTAKE_SPEED);
    }

    public void stopGripper() {
        gripperMotor.stopMotor();
    }

  
    public boolean hasGamePiece() {
        return !beamBreak.get(); // beam broken = object present
    }

    private double degreesToMotorRotations(double degrees) {
        // TODO: confirm reduction ratio from CAD
        double reduction = 59.0;
        return (degrees / 360.0) * reduction;
    }

    private double motorRotationsToDegrees(double rotations) {
        double reduction = 59.0;
        return (rotations / reduction) * 360.0;
    }

}
