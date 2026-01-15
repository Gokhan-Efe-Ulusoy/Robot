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

/**
 * Vision subsystem that manages multiple cameras for AprilTag detection and pose estimation.
 * Provides robot localization through vision measurements and target detection for auto-alignment.
 * 
 * <p>Features:
 * <ul>
 *   <li>Multi-camera support with hot-swapping capability</li>
 *   <li>Automatic pose estimation filtering and validation</li>
 *   <li>Thread-safe tag tracking for concurrent access</li>
 *   <li>Comprehensive telemetry and diagnostics</li>
 * </ul>
 */
public class Vision extends SubsystemBase {

    // Core dependencies
    private final DriveSubsystem drive;
    private final VisionIO[] cameras;
    private final VisionIOInputs[] inputs;
    private final AprilTagFieldLayout fieldLayout;

    // Thread-safe visible tags list (atomic replacement pattern)
    private volatile List<VisionTag> latestVisibleTags = new ArrayList<>();

    // Measurement history for future outlier detection
    private final LinkedList<VisionMeasurement> measurementHistory = new LinkedList<>();
    private static final int MAX_HISTORY_SIZE = 50;

    // Diagnostics and metrics
    private long lastMeasurementTimeMs = 0;
    private int consecutiveRejections = 0;
    private int totalMeasurementsAccepted = 0;
    private int totalMeasurementsRejected = 0;

    // Rejection reason tracking
    private String lastRejectionReason = "None";

    /**
     * Creates a new Vision subsystem.
     * 
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

        // Load AprilTag field layout
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
        // Create new tag list for thread-safe atomic replacement
        List<VisionTag> newTags = new ArrayList<>();

        // Process all cameras
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(inputs[i]);
            processCamera(inputs[i], newTags, i);
        }

        // Atomic replacement for thread safety
        latestVisibleTags = newTags;

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Processes vision data from a single camera.
     * Validates measurements and updates robot pose estimation if valid.
     * 
     * @param input The vision inputs from the camera
     * @param targetList The list to add visible tags to
     * @param cameraIndex Index of the camera for logging
     */
    private void processCamera(VisionIOInputs input, List<VisionTag> targetList, int cameraIndex) {
        // Check 1: Camera connected and has targets
        if (!input.connected) {
            handleRejection("Camera " + cameraIndex + " disconnected");
            return;
        }

        if (!input.hasTargets || input.estimatedPose == null) {
            handleRejection("No targets detected");
            return;
        }

        // Check 2: Pose ambiguity validation (CRITICAL FIX)
        if (input.poseAmbiguity > VisionConstants.MAX_POSE_AMBIGUITY) {
            handleRejection(String.format(
                "High ambiguity: %.3f > %.3f", 
                input.poseAmbiguity, 
                VisionConstants.MAX_POSE_AMBIGUITY
            ));
            return;
        }

        // Check 3: Field bounds validation
        if (!isPoseInFieldBounds(input.estimatedPose)) {
            handleRejection(String.format(
                "Out of bounds: (%.2f, %.2f)", 
                input.estimatedPose.getX(), 
                input.estimatedPose.getY()
            ));
            return;
        }

        // Check 4: Tag distance validation
        if (input.averageTagDistance > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            handleRejection(String.format(
                "Tag too far: %.2fm > %.2fm", 
                input.averageTagDistance, 
                VisionConstants.MAX_TAG_DISTANCE_METERS
            ));
            return;
        }

        // Check 5: Tag area validation (too small = unreliable)
        if (input.averageTagArea < VisionConstants.MIN_TAG_AREA) {
            handleRejection(String.format(
                "Tag too small: %.3f < %.3f", 
                input.averageTagArea, 
                VisionConstants.MIN_TAG_AREA
            ));
            return;
        }

        // All checks passed - accept measurement
        drive.addVisionMeasurement(
            input.estimatedPose,
            input.timestamp
        );

        // Extract visible tags for auto-alignment
        extractVisibleTags(input, targetList);

        // Record measurement in history
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

        // Update success metrics
        lastMeasurementTimeMs = System.currentTimeMillis();
        consecutiveRejections = 0;
        totalMeasurementsAccepted++;
        lastRejectionReason = "None";
    }

    /**
     * Extracts visible AprilTags from camera input and adds them to the target list.
     * 
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
     * Gets the current list of visible AprilTags.
     * Thread-safe for use by other subsystems.
     * 
     * @return Immutable copy of currently visible tags
     */
    public List<VisionTag> getVisibleTags() {
        return List.copyOf(latestVisibleTags);
    }

    /**
     * Finds the best reef target for auto-alignment based on current robot position.
     * Uses ReefTargetSelector to choose optimal target considering distance and heading.
     * 
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
     * Validates that a pose is within reasonable field boundaries.
     * Includes small tolerance for edge cases.
     * 
     * @param pose The pose to validate
     * @return true if pose is within field bounds plus tolerance
     */
    private boolean isPoseInFieldBounds(Pose2d pose) {
        final double TOLERANCE = 0.2; // 20cm tolerance (reduced from 50cm)
        
        double x = pose.getX();
        double y = pose.getY();
        
        return x >= -TOLERANCE 
            && x <= VisionConstants.FIELD_LENGTH_METERS + TOLERANCE
            && y >= -TOLERANCE 
            && y <= VisionConstants.FIELD_WIDTH_METERS + TOLERANCE;
    }

    /**
     * Handles measurement rejection and updates diagnostics.
     * 
     * @param reason Human-readable reason for rejection
     */
    private void handleRejection(String reason) {
        consecutiveRejections++;
        totalMeasurementsRejected++;
        lastRejectionReason = reason;
    }

    /**
     * Updates SmartDashboard telemetry for diagnostics and tuning.
     */
    private void updateTelemetry() {
        long timeSinceLastMeasurement = System.currentTimeMillis() - lastMeasurementTimeMs;
        
        // Core metrics
        SmartDashboard.putNumber("Vision/VisibleTagCount", latestVisibleTags.size());
        SmartDashboard.putNumber("Vision/TimeSinceLastMeasurementMs", timeSinceLastMeasurement);
        SmartDashboard.putNumber("Vision/ConsecutiveRejections", consecutiveRejections);
        
        // Success/failure tracking
        SmartDashboard.putNumber("Vision/TotalAccepted", totalMeasurementsAccepted);
        SmartDashboard.putNumber("Vision/TotalRejected", totalMeasurementsRejected);
        
        // Acceptance rate
        int total = totalMeasurementsAccepted + totalMeasurementsRejected;
        double acceptanceRate = total > 0 ? (100.0 * totalMeasurementsAccepted / total) : 0.0;
        SmartDashboard.putNumber("Vision/AcceptanceRate", acceptanceRate);
        
        // Diagnostics
        SmartDashboard.putString("Vision/LastRejectionReason", lastRejectionReason);
        SmartDashboard.putBoolean("Vision/HealthOK", consecutiveRejections < 50);
        
        // Auto-align status
        if (drive != null) {
            Optional<Pose2d> reefTarget = getBestReefTarget(drive.getPose());
            SmartDashboard.putBoolean("Vision/HasReefTarget", reefTarget.isPresent());
            reefTarget.ifPresent(target -> {
                SmartDashboard.putNumber("Vision/ReefTargetX", target.getX());
                SmartDashboard.putNumber("Vision/ReefTargetY", target.getY());
                SmartDashboard.putNumber("Vision/ReefTargetAngle", target.getRotation().getDegrees());
            });
        }

        // Tag ID list for debugging
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
     * Gets measurement history for analysis or outlier detection.
     * 
     * @return Immutable copy of measurement history
     */
    public List<VisionMeasurement> getMeasurementHistory() {
        return List.copyOf(measurementHistory);
    }

    /**
     * Clears all measurement history and resets diagnostics.
     * Useful for testing or match start.
     */
    public void resetDiagnostics() {
        measurementHistory.clear();
        totalMeasurementsAccepted = 0;
        totalMeasurementsRejected = 0;
        consecutiveRejections = 0;
        lastRejectionReason = "Reset";
        DriverStation.reportWarning("Vision diagnostics reset", false);
    }

    /**
     * Gets the AprilTag field layout.
     * 
     * @return Optional containing the field layout, or empty if failed to load
     */
    public Optional<AprilTagFieldLayout> getFieldLayout() {
        return Optional.ofNullable(fieldLayout);
    }

    /**
     * Checks if vision system is healthy and providing measurements.
     * 
     * @return true if recent measurements are being accepted
     */
    public boolean isHealthy() {
        long timeSinceLastMeasurement = System.currentTimeMillis() - lastMeasurementTimeMs;
        return consecutiveRejections < 50 && timeSinceLastMeasurement < 1000;
    }

    /**
     * Container for a single vision measurement with metadata.
     */
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

        /**
         * Legacy constructor for backward compatibility.
         */
        public VisionMeasurement(Pose2d pose, double timestamp) {
            this(pose, timestamp, -1, 0, 1.0);
        }
    }
}