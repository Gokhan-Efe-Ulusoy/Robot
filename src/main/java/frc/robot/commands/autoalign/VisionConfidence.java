package frc.robot.commands.autoalign;

public class VisionConfidence {

    private static final double MAX_AMBIGUITY = 0.25;
    private static final double MAX_AGE_SEC = 0.4;

    public static boolean isUsable(
            double ambiguity,
            double timestamp,
            double now,
            int tagCount
    ) {
        if (tagCount >= 2 && ambiguity < 0.4) return true;

        return ambiguity <= MAX_AMBIGUITY
                && (now - timestamp) <= MAX_AGE_SEC;
    }
}
