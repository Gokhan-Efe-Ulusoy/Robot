package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.CoralCradleConstants;

public class CoralCradleSubsystem extends SubsystemBase {

    private final Servo cradleServo;
    private CradleState currentState = CradleState.CLOSED;

    public enum CradleState {
        OPEN,
        CLOSED
    }

    public CoralCradleSubsystem() {
        cradleServo = new Servo(CoralCradleConstants.SERVO_CHANNEL);
        cradleServo.setBoundsMicroseconds(2.0, 1.8, 1.5,1.2, 1.0);
        close(); // safe default
    }

    public void setState(CradleState state) {
        switch (state) {
            case OPEN -> open();
            case CLOSED -> close();
        }
    }

    public void open() {
        cradleServo.set(
            MathUtil.clamp(
                CoralCradleConstants.OPEN_POSITION,
                0.0,
                1.0
            )
        );
        currentState = CradleState.OPEN;
    }

    public void close() {
        cradleServo.set(
            MathUtil.clamp(
                CoralCradleConstants.CLOSED_POSITION,
                0.0,
                1.0
            )
        );
        currentState = CradleState.CLOSED;
    }

    public boolean isClosed(){
        return currentState == CradleState.CLOSED;
    }
}
