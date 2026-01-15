package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {

    private final DriveSubsystem drive;
    private final VisionIO[] cameras;
    private final VisionIOInputs[] inputs;
    private final AprilTagFieldLayout fieldLayout;

    private volatile List<VisionTag> latestVisibleTags = new ArrayList<>();

    private final LinkedList<VisionMeasurement> measurementHistory = new LinkedList<>();
    private static final int MAX_HISTORY_SIZE = 50;

    private long lastMeasurementTimeMs = 0;
    private int consecutiveRejections = 0;
    private int totalMeasurementsAccepted = 0;
    private int totalMeasurementsRejected = 0;

    private String lastRejectionReason = "None";

    public Vision(DriveSubsystem drive, VisionIO... visionIOs) {
        this.drive = drive;
        this.cameras = visionIOs;
        this.inputs = new VisionIOInputs[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            inputs[i] = new VisionIOInputs();
        }

        AprilTagFieldLayout layout;
        try {
            layout = new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getAbsolutePath(),
                    "2025-reefscape.json"
                )
            );
            DriverStation.reportWarning("Vision: AprilTag layout loaded successfully", false);
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load 2025 Reefscape AprilTag layout",
                e.getStackTrace()
            );
            layout = null;
        }
        fieldLayout = layout;
    }

    @Override
    public void periodic() {
        List<VisionTag> newTags = new ArrayList<>();

        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(inputs[i]);
            processCamera(inputs[i], newTags, i);
        }

        latestVisibleTags = newTags;
        updateTelemetry();
    }

    private void processCamera(VisionIOInputs input, List<VisionTag> targetList, int cameraIndex) {
        if (!input.connected) {
            handleRejection("Camera " + cameraIndex + " disconnected");
            return;
        }

        if (!input.hasTargets || input.estimatedPose == null) {
            handleRejection("No targets detected");
            return;
        }

        if (input.poseAmbiguity > VisionConstants.MAX_POSE_AMBIGUITY) {
            handleRejection(String.format(
                "High ambiguity: %.3f > %.3f",
                input.poseAmbiguity,
                VisionConstants.MAX_POSE_AMBIGUITY
            ));
            return;
        }

        if (!isPoseInFieldBounds(input.estimatedPose)) {
            handleRejection("Pose out of field bounds");
            return;
        }

        if (input.averageTagDistance > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            handleRejection("Tag too far");
            return;
        }

        if (input.averageTagArea < VisionConstants.MIN_TAG_AREA) {
            handleRejection("Tag area too small");
            return;
        }

        drive.addVisionMeasurement(
            input.estimatedPose,
            input.timestamp
        );

        extractVisibleTags(input, targetList);

        measurementHistory.addFirst(
            new VisionMeasurement(
                input.estimatedPose,
                input.timestamp,
                cameraIndex,
                input.tagCount,
                input.poseAmbiguity
            )
        );

        if (measurementHistory.size() > MAX_HISTORY_SIZE) {
            measurementHistory.removeLast();
        }

        lastMeasurementTimeMs = System.currentTimeMillis();
        consecutiveRejections = 0;
        totalMeasurementsAccepted++;
        lastRejectionReason = "None";
    }

    private void extractVisibleTags(VisionIOInputs input, List<VisionTag> targetList) {
        if (fieldLayout == null) return;

        for (int id : input.visibleTagIds) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
            tagPose.ifPresent(pose ->
                targetList.add(new VisionTag(id, pose.toPose2d()))
            );
        }
    }

    public List<VisionTag> getVisibleTags() {
        return List.copyOf(latestVisibleTags);
    }

    private boolean isPoseInFieldBounds(Pose2d pose) {
        final double TOLERANCE = 0.2;

        return pose.getX() >= -TOLERANCE
            && pose.getX() <= VisionConstants.FIELD_LENGTH_METERS + TOLERANCE
            && pose.getY() >= -TOLERANCE
            && pose.getY() <= VisionConstants.FIELD_WIDTH_METERS + TOLERANCE;
    }

    private void handleRejection(String reason) {
        consecutiveRejections++;
        totalMeasurementsRejected++;
        lastRejectionReason = reason;
    }

    private void updateTelemetry() {
        long timeSinceLastMeasurement =
            System.currentTimeMillis() - lastMeasurementTimeMs;

        SmartDashboard.putNumber("Vision/VisibleTagCount", latestVisibleTags.size());
        SmartDashboard.putNumber("Vision/ConsecutiveRejections", consecutiveRejections);
        SmartDashboard.putString("Vision/LastRejectionReason", lastRejectionReason);
        SmartDashboard.putBoolean("Vision/Healthy", isHealthy());
    }

    public List<VisionMeasurement> getMeasurementHistory() {
        return List.copyOf(measurementHistory);
    }

    public boolean isHealthy() {
        long timeSinceLastMeasurement =
            System.currentTimeMillis() - lastMeasurementTimeMs;

        return consecutiveRejections < 50 && timeSinceLastMeasurement < 1000;
    }

    public Optional<AprilTagFieldLayout> getFieldLayout() {
        return Optional.ofNullable(fieldLayout);
    }

    public static class VisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public final int cameraIndex;
        public final int tagCount;
        public final double ambiguity;

        public VisionMeasurement(
            Pose2d pose,
            double timestamp,
            int cameraIndex,
            int tagCount,
            double ambiguity
        ) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.cameraIndex = cameraIndex;
            this.tagCount = tagCount;
            this.ambiguity = ambiguity;
        }
    }
}
