package frc.robot.commands.autoalign;

public final class AutoAlignConstants {

    private AutoAlignConstants() {}

    /* ---------------- TRANSLATION PID ---------------- */
    public static final double TRANSLATION_kP = 2.5;
    public static final double TRANSLATION_kI = 0.0;
    public static final double TRANSLATION_kD = 0.1;

    public static final double MAX_TRANSLATION_VEL = 2.5;   // m/s
    public static final double MAX_TRANSLATION_ACCEL = 3.0; // m/s^2

    /* ---------------- ROTATION PID ---------------- */
    public static final double ROTATION_kP = 5.0;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.15;

    public static final double MAX_ROTATION_VEL =
            Math.toRadians(360);
    public static final double MAX_ROTATION_ACCEL =
            Math.toRadians(720);

    /* ---------------- FSM THRESHOLDS ---------------- */
    public static final double APPROACH_DISTANCE = 0.7;

    /* ---------------- ALIGN TOLERANCES ---------------- */
    public static final double POSITION_TOLERANCE = 0.05;
    public static final double ROTATION_TOLERANCE =
            Math.toRadians(2.0);

    /* ---------------- VISION CONFIDENCE ---------------- */
    public static final double TELEOP_MAX_TARGET_AGE = 0.5;
    public static final double AUTO_MAX_TARGET_AGE = 0.2;
    public static final double MAX_AMBIGUITY = 0.25;
}
