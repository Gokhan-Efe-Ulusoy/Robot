package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final double angleOffsetRotations;
    
    private final SimpleMotorFeedforward driveFeedforward =
    new SimpleMotorFeedforward(0.2, 2.5); // example values

    public SwerveModule(int driveID, int steerID, double angleOffsetRotations) {
        this.driveMotor = new TalonFX(driveID);
        this.steerMotor = new TalonFX(steerID);
        this.angleOffsetRotations = angleOffsetRotations;

        configureCurrentLimits();

        setBrakeMode(true);

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimized =
                SwerveModuleState.optimize(
                        desiredState,
                        getAngle()
                );

        driveMotor.setControl(

                // Assuming m/s
                new VelocityVoltage(
                        optimized.speedMetersPerSecond
                ).withFeedForward(
                    driveFeedforward.calculate(
                        optimized.speedMetersPerSecond
                    )
                )
        );

        steerMotor.setControl(
                new PositionVoltage(
                        optimized.angle.getRotations() + angleOffsetRotations
                )
        );
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                steerMotor.getPosition().getValue() - angleOffsetRotations
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.getVelocity().getValue(),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValue(),
                getAngle()
        );
    }
    private void configureCurrentLimits() {

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit =
                frc.robot.Constants.DriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentThreshold =
                frc.robot.Constants.DriveConstants.DRIVE_SUPPLY_CURRENT_THRESHOLD;
        driveConfig.CurrentLimits.SupplyTimeThreshold =
                frc.robot.Constants.DriveConstants.DRIVE_SUPPLY_TIME_THRESHOLD;
        
        driveConfig.Voltage.PeakForwardVoltage =
                frc.robot.Constants.DriveConstants.NOMINAL_VOLTAGE;
        driveConfig.Voltage.PeakReverseVoltage =
                -frc.robot.Constants.DriveConstants.NOMINAL_VOLTAGE;
    
        driveMotor.getConfigurator().apply(driveConfig);

    
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit =
                frc.robot.Constants.DriveConstants.STEER_SUPPLY_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentThreshold =
                frc.robot.Constants.DriveConstants.STEER_SUPPLY_CURRENT_THRESHOLD;
        steerConfig.CurrentLimits.SupplyTimeThreshold =
                frc.robot.Constants.DriveConstants.STEER_SUPPLY_TIME_THRESHOLD;
        
        steerConfig.Voltage.PeakForwardVoltage =
                frc.robot.Constants.DriveConstants.NOMINAL_VOLTAGE;
        steerConfig.Voltage.PeakReverseVoltage =
                -frc.robot.Constants.DriveConstants.NOMINAL_VOLTAGE;
    
        steerMotor.getConfigurator().apply(steerConfig);

    }

    public void setBrakeMode(boolean enable) {
        NeutralModeValue mode = enable
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        driveMotor.setNeutralMode(mode);
        steerMotor.setNeutralMode(mode);
    }


}
