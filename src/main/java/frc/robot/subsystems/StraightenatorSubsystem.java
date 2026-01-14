package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StraightenatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class StraightenatorSubsystem extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final Timer unjamTimer = new Timer();
    private boolean unjamming = false;

    public StraightenatorSubsystem() {
        leftMotor = new TalonFX(StraightenatorConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(StraightenatorConstants.RIGHT_MOTOR_ID);

        configureMotor(leftMotor);
        configureMotor(rightMotor);
    }

    private void configureMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentThreshold = 45.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;

        motor.getConfigurator().apply(config);

        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        if (unjamming) {
            if (unjamTimer.hasElapsed(StraightenatorConstants.UNJAM_TIME)) {
                unjamming = false;
                unjamTimer.stop();
                unjamTimer.reset();
                stop();
            }
            return;
        }

        if (isJammed()) {
            startUnjam();
        }
    }

    public void runIn() {
        leftMotor.set(StraightenatorConstants.IN_SPEED);
        rightMotor.set(StraightenatorConstants.IN_SPEED);
    }

    public void runOut() {
        leftMotor.set(StraightenatorConstants.OUT_SPEED);
        rightMotor.set(StraightenatorConstants.OUT_SPEED);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    private boolean isJammed() {
        return leftMotor.getSupplyCurrent().getValue() >
                    StraightenatorConstants.JAM_CURRENT
            || rightMotor.getSupplyCurrent().getValue() >
                    StraightenatorConstants.JAM_CURRENT;
    }

    private void startUnjam() {
        unjamming = true;
        unjamTimer.restart();

        leftMotor.set(StraightenatorConstants.UNJAM_SPEED);
        rightMotor.set(-StraightenatorConstants.UNJAM_SPEED);
    }
}

