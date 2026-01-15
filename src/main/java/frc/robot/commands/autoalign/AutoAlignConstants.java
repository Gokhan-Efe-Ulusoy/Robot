package frc.robot.commands.autoalign;

public final class AutoAlignConstants {

    private AutoAlignConstants() {}

    public static final double X_KP = 3.5;
    public static final double X_KD = 0.15;

    public static final double Y_KP = 3.5;
    public static final double Y_KD = 0.15;

    public static final double THETA_KP = 5.0;
    public static final double THETA_KD = 0.25;

    public static final double MAX_SPEED = 3.0;
    public static final double MAX_ACCEL = 4.0;

    public static final double MAX_OMEGA = 6.0;
    public static final double MAX_ALPHA = 10.0;

    public static final double POSITION_TOLERANCE = 0.05;
    public static final double ANGLE_TOLERANCE_RAD = Math.toRadians(2.0);

    public static final double AUTON_TIMEOUT_SEC = 2.5;
}
