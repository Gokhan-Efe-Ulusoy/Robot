package frc.robot.commands.autoalign;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionTag;

public final class ReefTargetSelector {

    // ---------------- CONSTANTS ----------------

    private static final List<Integer> BLUE_REEF_TAGS =
            List.of(17, 18, 19, 20, 21, 22);

    private static final List<Integer> RED_REEF_TAGS =
            List.of(6, 7, 8, 9, 10, 11);

    // Robotun reef’e yaklaşırken duracağı offset
    private static final Transform2d ALIGN_OFFSET =
            new Transform2d(
                    0.6,                       // ileri (metre)
                    0.0,                       // yanal
                    Rotation2d.fromDegrees(180)
            );

    private ReefTargetSelector() {
        // static utility class
    }

    // ---------------- PUBLIC API ----------------

    public static Optional<Pose2d> selectBestReefTarget(
            Pose2d robotPose,
            List<VisionTag> visibleTags) {

        if (visibleTags == null || visibleTags.isEmpty()) {
            return Optional.empty();
        }

        Alliance alliance =
                DriverStation.getAlliance().orElse(Alliance.Blue);

        List<Integer> validReefTags =
                alliance == Alliance.Blue ? BLUE_REEF_TAGS : RED_REEF_TAGS;

        return visibleTags.stream()

                // Reef tag mi?
                .filter(tag -> validReefTags.contains(tag.getId()))

                // En iyi hedefi seç
                .min(Comparator.comparingDouble(
                        tag -> computeSelectionScore(robotPose, tag)))

                // Align pozisyonuna çevir
                .map(ReefTargetSelector::computeAlignPose);
    }

    // ---------------- INTERNAL LOGIC ----------------

    private static double computeSelectionScore(
            Pose2d robotPose,
            VisionTag tag) {

        Pose2d tagPose = tag.getFieldPose();

        double distance =
                robotPose.getTranslation()
                        .getDistance(tagPose.getTranslation());

        double headingError =
                Math.abs(
                        robotPose.getRotation()
                                .minus(tagPose.getRotation())
                                .getDegrees()
                );

        // Basit ama çok stabil bir skor fonksiyonu
        return distance + headingError * 0.01;
    }

    private static Pose2d computeAlignPose(VisionTag tag) {
        return tag.getFieldPose().transformBy(ALIGN_OFFSET);
    }
}
