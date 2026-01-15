package frc.robot.commands.autoalign;

public class VisionConfidence {

    public final boolean hasTarget;
    public final double ambiguity;
    public final double distance;
    public final double timestamp;

    public VisionConfidence(
            boolean hasTarget,
            double ambiguity,
            double distance,
            double timestamp
    ) {
        this.hasTarget = hasTarget;
        this.ambiguity = ambiguity;
        this.distance = distance;
        this.timestamp = timestamp;
    }

    public boolean isUsable(double now, double maxAge) {
        return hasTarget
                && ambiguity <= AutoAlignConstants.MAX_AMBIGUITY
                && (now - timestamp) <= maxAge;
    }
}
