package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private double targetHeightMeters = 0.0;
    private boolean homed = false;

    public enum ElevatorPosition {
        HOME,
        L1,
        L2,
        L3,
        L4,
        NET,
        PROCESSOR
    }

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID);

        configureMotor(leftMotor);
        configureMotor(rightMotor);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    private void configureMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentThreshold = 60.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.2;

        config.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration =
                ElevatorConstants.ACCELERATION;

        // Slot 0 PID estimated
        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kD = ElevatorConstants.kD;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                metersToMotorRotations(ElevatorConstants.MAX_HEIGHT_METERS);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPosition(ElevatorPosition position) {
        switch (position) {
            case HOME -> setHeightMeters(ElevatorConstants.HOME_HEIGHT);
            case L1 -> setHeightMeters(ElevatorConstants.L1_HEIGHT);
            case L2 -> setHeightMeters(ElevatorConstants.L2_HEIGHT);
            case L3 -> setHeightMeters(ElevatorConstants.L3_HEIGHT);
            case L4 -> setHeightMeters(ElevatorConstants.L4_HEIGHT);
            case NET -> setHeightMeters(ElevatorConstants.NET_HEIGHT);
            case PROCESSOR -> setHeightMeters(ElevatorConstants.PROCESSOR_HEIGHT);
        }
    }

    public void setHeightMeters(double meters) {
        
        meters = MathUtil.clamp(
            meters,
            ElevatorConstants.MIN_HEIGHT_METERS,
            ElevatorConstants.MAX_HEIGHT_METERS
        );

        targetHeightMeters = meters;

        double rotations = metersToMotorRotations(meters);
        leftMotor.setControl(new MotionMagicVoltage(rotations));
    }

    public void home() {

        // later, we should add limit switch
        leftMotor.setPosition(0);
        homed = true;
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean atTarget() {
        return Math.abs(getHeightMeters() - targetHeightMeters) < 0.02;
    }

    public double getHeightMeters() {
        return motorRotationsToMeters(
                leftMotor.getPosition().getValue()
        );
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    private double metersToMotorRotations(double meters) {
        // assumption
        double pulleyCircumference = 0.05;
        return (meters / pulleyCircumference) * ElevatorConstants.GEAR_RATIO;
    }

    private double motorRotationsToMeters(double rotations) {
        double pulleyCircumference = 0.05;
        return (rotations / ElevatorConstants.GEAR_RATIO) * pulleyCircumference;
    }
    
}
