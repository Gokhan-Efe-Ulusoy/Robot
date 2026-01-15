package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autoalign.ReefTargetSelector;
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

    /**
     * @param drive The drive subsystem for pose estimation updates
     * @param visionIOs Variable number of vision IO implementations (cameras)
     */
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
                "Failed to load 2025 Reefscape AprilTag layout. Vision will operate without field layout.",
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

    /**
     * @param input The vision inputs from the camera
     * @param targetList The list to add visible tags to
     * @param cameraIndex Index of the camera for logging
     */
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
            handleRejection(String.format(
                "Out of bounds: (%.2f, %.2f)", 
                input.estimatedPose.getX(), 
                input.estimatedPose.getY()
            ));
            return;
        }

        if (input.averageTagDistance > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            handleRejection(String.format(
                "Tag too far: %.2fm > %.2fm", 
                input.averageTagDistance, 
                VisionConstants.MAX_TAG_DISTANCE_METERS
            ));
            return;
        }

        if (input.averageTagArea < VisionConstants.MIN_TAG_AREA) {
            handleRejection(String.format(
                "Tag too small: %.3f < %.3f", 
                input.averageTagArea, 
                VisionConstants.MIN_TAG_AREA
            ));
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

    /**
     * @param input The vision inputs containing tag IDs
     * @param targetList The list to add detected tags to
     */
    private void extractVisibleTags(VisionIOInputs input, List<VisionTag> targetList) {
        if (fieldLayout == null) {
            return;
        }

        for (int id : input.visibleTagIds) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
            tagPose.ifPresent(pose ->
                targetList.add(new VisionTag(id, pose.toPose2d()))
            );
        }
    }

    /**
     * @return Immutable copy of currently visible tags
     */
    public List<VisionTag> getVisibleTags() {
        return List.copyOf(latestVisibleTags);
    }

    /**
     * @param robotPose Current robot pose on the field
     * @return Optional containing the target pose, or empty if no valid target found
     */
    public Optional<Pose2d> getBestReefTarget(Pose2d robotPose) {
        return ReefTargetSelector.selectBestReefTarget(
            robotPose, 
            getVisibleTags()
        );
    }

    /**
     * @param pose The pose to validate
     * @return true if pose is within field bounds plus tolerance
     */
    private boolean isPoseInFieldBounds(Pose2d pose) {
        final double TOLERANCE = 0.2;
        
        double x = pose.getX();
        double y = pose.getY();
        
        return x >= -TOLERANCE 
            && x <= VisionConstants.FIELD_LENGTH_METERS + TOLERANCE
            && y >= -TOLERANCE 
            && y <= VisionConstants.FIELD_WIDTH_METERS + TOLERANCE;
    }

    /**
     * @param reason Human-readable reason for rejection
     */
    private void handleRejection(String reason) {
        consecutiveRejections++;
        totalMeasurementsRejected++;
        lastRejectionReason = reason;
    }


    private void updateTelemetry() {
        long timeSinceLastMeasurement = System.currentTimeMillis() - lastMeasurementTimeMs;
        
        SmartDashboard.putNumber("Vision/VisibleTagCount", latestVisibleTags.size());
        SmartDashboard.putNumber("Vision/TimeSinceLastMeasurementMs", timeSinceLastMeasurement);
        SmartDashboard.putNumber("Vision/ConsecutiveRejections", consecutiveRejections);
        
        SmartDashboard.putNumber("Vision/TotalAccepted", totalMeasurementsAccepted);
        SmartDashboard.putNumber("Vision/TotalRejected", totalMeasurementsRejected);
        
        int total = totalMeasurementsAccepted + totalMeasurementsRejected;
        double acceptanceRate = total > 0 ? (100.0 * totalMeasurementsAccepted / total) : 0.0;
        SmartDashboard.putNumber("Vision/AcceptanceRate", acceptanceRate);
        
        SmartDashboard.putString("Vision/LastRejectionReason", lastRejectionReason);
        SmartDashboard.putBoolean("Vision/HealthOK", consecutiveRejections < 50);
        
        if (drive != null) {
            Optional<Pose2d> reefTarget = getBestReefTarget(drive.getPose());
            SmartDashboard.putBoolean("Vision/HasReefTarget", reefTarget.isPresent());
            reefTarget.ifPresent(target -> {
                SmartDashboard.putNumber("Vision/ReefTargetX", target.getX());
                SmartDashboard.putNumber("Vision/ReefTargetY", target.getY());
                SmartDashboard.putNumber("Vision/ReefTargetAngle", target.getRotation().getDegrees());
            });
        }

        if (!latestVisibleTags.isEmpty()) {
            StringBuilder tagIds = new StringBuilder();
            for (VisionTag tag : latestVisibleTags) {
                tagIds.append(tag.getId()).append(" ");
            }
            SmartDashboard.putString("Vision/VisibleTagIDs", tagIds.toString().trim());
        } else {
            SmartDashboard.putString("Vision/VisibleTagIDs", "None");
        }
    }

    /**
     * @return Immutable copy of measurement history
     */
    public List<VisionMeasurement> getMeasurementHistory() {
        return List.copyOf(measurementHistory);
    }
    public void resetDiagnostics() {
        measurementHistory.clear();
        totalMeasurementsAccepted = 0;
        totalMeasurementsRejected = 0;
        consecutiveRejections = 0;
        lastRejectionReason = "Reset";
        DriverStation.reportWarning("Vision diagnostics reset", false);
    }

    /**
     * @return Optional containing the field layout, or empty if failed to load
     */
    public Optional<AprilTagFieldLayout> getFieldLayout() {
        return Optional.ofNullable(fieldLayout);
    }

    /**
     * @return true if recent measurements are being accepted
     */
    public boolean isHealthy() {
        long timeSinceLastMeasurement = System.currentTimeMillis() - lastMeasurementTimeMs;
        return consecutiveRejections < 50 && timeSinceLastMeasurement < 1000;
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
        public VisionMeasurement(Pose2d pose, double timestamp) {
            this(pose, timestamp, -1, 0, 1.0);
        }
    }
}