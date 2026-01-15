package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Immutable vision tag representation
 * Auto-align & reef selector safe
 */
public final class VisionTag {

    private final int id;
    private final Pose2d fieldPose;

    public VisionTag(int id, Pose2d fieldPose) {
        this.id = id;
        this.fieldPose = fieldPose;
    }

    public int getId() {
        return id;
    }

    public Pose2d getFieldPose() {
        return fieldPose;
    }
}
