package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX intakeMotor;
    private final DigitalInput coralSensor;
    private final DigitalInput algaeSensor;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);
        configureMotor();
        coralSensor = new DigitalInput(IntakeConstants.CORAL_SENSOR_DIO);
        algaeSensor = new DigitalInput(IntakeConstants.ALGAE_SENSOR_DIO);
    }
    private void configureMotor() {

        var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 25.0;
        config.CurrentLimits.SupplyCurrentThreshold = 35.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
    
        intakeMotor.getConfigurator().apply(config);
    
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }    

    public void intake() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void eject() {
        intakeMotor.set(IntakeConstants.EJECT_SPEED);
    
    }
    public void stop() {
        intakeMotor.stopMotor();
    }

    public void hold() {
        intakeMotor.set(IntakeConstants.HOLD_SPEED); 
    }

    public boolean isRunning() {
        return Math.abs(intakeMotor.getVelocity().getValue()) > 1.0;
    }    
    
    public void intakeCoral() {
        intake();
    }


    public void intakeAlgae() {
        intake();
    }


    public void outtake() {
        eject();
    }


    public void hold() {
        intakeMotor.set(0.10); // 10% power
    }


    public boolean hasCoral() {
        return !coralSensor.get();
    }


    public boolean hasAlgae() {
        return !algaeSensor.get();
    }

    @Override
    public void periodic() {
        if (hasCoral() || hasAlgae()) {
            hold();
        }
    }
}

