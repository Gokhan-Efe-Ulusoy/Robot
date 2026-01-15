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

    private final LinkedList<VisionMeasurement> measurementHistory = new LinkedList<>();
    private static final int MAX_HISTORY_SIZE = 50;

    private final List<VisionTag> latestVisibleTags = new ArrayList<>();

    private long lastMeasurementTimeMs = 0;
    private int consecutiveRejections = 0;

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
        latestVisibleTags.clear();

        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(inputs[i]);
            processCamera(inputs[i]);
        }

        SmartDashboard.putNumber(
            "Vision/TimeSinceLastMeasurementMs",
            System.currentTimeMillis() - lastMeasurementTimeMs
        );
    }

    private void processCamera(VisionIOInputs input) {
        if (!input.hasTargets || input.estimatedPose == null) {
            consecutiveRejections++;
            return;
        }

        if (!isPoseInFieldBounds(input.estimatedPose)) {
            consecutiveRejections++;
            return;
        }

        drive.addVisionMeasurement(
            input.estimatedPose,
            input.timestamp
        );

        extractVisibleTags(input);

        measurementHistory.addFirst(
            new VisionMeasurement(input.estimatedPose, input.timestamp)
        );
        if (measurementHistory.size() > MAX_HISTORY_SIZE) {
            measurementHistory.removeLast();
        }

        lastMeasurementTimeMs = System.currentTimeMillis();
        consecutiveRejections = 0;
    }

    private void extractVisibleTags(VisionIOInputs input) {
        if (fieldLayout == null) return;

        for (int id : input.visibleTagIds) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
            tagPose.ifPresent(pose ->
                latestVisibleTags.add(
                    new VisionTag(id, pose.toPose2d())
                )
            );
        }
    }

    public List<VisionTag> getVisibleTags() {
        return List.copyOf(latestVisibleTags);
    }

    private boolean isPoseInFieldBounds(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -0.5 && x <= VisionConstants.FIELD_LENGTH_METERS + 0.5
            && y >= -0.5 && y <= VisionConstants.FIELD_WIDTH_METERS + 0.5;
    }

    public static class VisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;

        public VisionMeasurement(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    public Optional<Pose2d> getBestReefTarget(Pose2d robotPose) {
        return Optional.empty();
    }

}
