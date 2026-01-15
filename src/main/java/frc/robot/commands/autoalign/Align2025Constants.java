package frc.robot.commands.autoalign;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Align2025Constants {

    private Align2025Constants() {}

    public static final Transform2d REEF_OFFSET =
        new Transform2d(
            new Translation2d(0.55, 0.0),
            new Rotation2d()
        );

    public static final double POSITION_TOLERANCE_METERS = 0.04;
    public static final double ANGLE_TOLERANCE_DEG = 2.0;

    public static final double APPROACH_MAX_SPEED = 2.0;
    public static final double FINAL_MAX_SPEED = 0.4;

    public static final double APPROACH_MAX_OMEGA = Math.PI;
    public static final double FINAL_MAX_OMEGA = Math.PI / 2.0;
}
