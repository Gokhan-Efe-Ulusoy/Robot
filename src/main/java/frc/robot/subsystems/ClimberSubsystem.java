package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;
    private boolean climberLocked = false;

    public ClimberSubsystem() {
        climberMotor = new TalonFX(ClimberConstants.MOTOR_ID);

        configureCurrentLimits();

        climberMotor.setNeutralMode(NeutralModeValue.Brake);

        topLimit = new DigitalInput(ClimberConstants.TOP_LIMIT_DIO);
        bottomLimit = new DigitalInput(ClimberConstants.BOTTOM_LIMIT_DIO);
    }
    private void configureCurrentLimits() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentThreshold = ClimberConstants.SUPPLY_CURRENT_THRESHOLD;
        config.CurrentLimits.SupplyTimeThreshold = ClimberConstants.SUPPLY_TIME_THRESHOLD;

        climberMotor.getConfigurator().apply(config);
    }
    private double clamp(double value) {
        return MathUtil.clamp(value, -1.0, 1.0);
    }    


    //Control Methods

    public void climbUp() {
        if (isStalled()) {
            hold();
            return;
        }
    
        if (isAtTop() || isPastTopSoftLimit()) {
            hold();
            climberLocked = true;
            hold();
            return;
        } else {
            climberMotor.set(clamp(ClimberConstants.UP_SPEED));
        }
    }
    
    public void climbDown() {
        if (isStalled()) {
            hold();
            return;
        }
    
        if (climberLocked) {
            hold();
            return;
        }
    
        if (isAtBottom() || isPastBottomSoftLimit()) {
            hold();
        } else {
            climberMotor.set(clamp(ClimberConstants.DOWN_SPEED));
        }
    }    

    public void hold() {
        climberMotor.set(0.0);
    }

    public void unlockClimber() {
        climberLocked = false;
    }    

    //

    public boolean isPastTopSoftLimit() {
        return climberMotor.getPosition().getValue() >=
               ClimberConstants.TOP_SOFT_LIMIT_ROTATIONS;
    }
    
    public boolean isPastBottomSoftLimit() {
        return climberMotor.getPosition().getValue() <=
               ClimberConstants.BOTTOM_SOFT_LIMIT_ROTATIONS;
    }
    public boolean isStalled() {
        return climberMotor.getSupplyCurrent().getValue() >
               ClimberConstants.STALL_CURRENT;
    }    
    
    public boolean isClimbing() {
        return Math.abs(climberMotor.getVelocity().getValue()) > 1.0;
    }

    public boolean isAtTop() {
      return !topLimit.get();
    }

    public boolean isAtBottom() {
      return !bottomLimit.get();
    }
}
