package frc.robot.commands.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionTag;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * ReefTargetSelector
 *
 * Vision subsystem'den gelen VisionTag'leri kullanarak
 * robot için en uygun reef align hedefini seçer.
 *
 * Bu sınıf:
 * - Vision'a bağımlı değildir
 * - Drive'a bağımlı değildir
 * - Sadece geometrik karar verir
 */
public final class ReefTargetSelector {

    private ReefTargetSelector() {}

    public static Optional<Pose2d> selectBestReefTarget(
            Pose2d robotPose,
            List<VisionTag> visibleTags
    ) {
        if (visibleTags.isEmpty()) {
            return Optional.empty();
        }

        return visibleTags.stream()
                .filter(ReefTargetSelector::isReefTag)
                .min(Comparator.comparingDouble(
                        tag -> distance(
                                robotPose.getTranslation(),
                                tag.getFieldPose().getTranslation()
                        )
                ))
                .map(ReefTargetSelector::computeTargetPose);
    }

    /**
     * 2025 Reefscape reef tag ID filtresi
     */
    private static boolean isReefTag(VisionTag tag) {
        int id = tag.getId();
        return id >= 17 && id <= 24; // Field layout'a göre doğrulanmalı
    }

    /**
     * AprilTag pose'undan robotun gitmesi gereken align pose'unu üretir
     */
    private static Pose2d computeTargetPose(VisionTag tag) {

        final double FORWARD_OFFSET_METERS = 0.65;

        Pose2d tagPose = tag.getFieldPose();

        Translation2d offset = new Translation2d(
                -FORWARD_OFFSET_METERS,
                0.0
        ).rotateBy(tagPose.getRotation());

        Translation2d targetTranslation =
                tagPose.getTranslation().plus(offset);

        return new Pose2d(
                targetTranslation,
                tagPose.getRotation()
        );
    }

    private static double distance(Translation2d a, Translation2d b) {
        return a.getDistance(b);
    }
}
