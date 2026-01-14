package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


import java.util.Optional;

public class Vision extends SubsystemBase {
    private final AprilTagFieldLayout fieldLayout;
    private final VisionIO[] cameras;
    private final VisionIOInputs[] inputs;
    
    private static final double MAX_POSE_AMBIGUITY = 0.2;
    private static final double MAX_TAG_DISTANCE_METERS = 4.5;
    private static final double MIN_TAG_AREA = 0.1;
    
    private static final double SINGLE_TAG_STD_DEV_BASE = 0.8;
    private static final double MULTI_TAG_STD_DEV_BASE = 0.3;
    
    private boolean enableVisionUpdates = true;
    
    public Vision(VisionIO... visionIOs) {
        this.cameras = visionIOs;
        this.inputs = new VisionIOInputs[cameras.length];
        
        for (int i = 0; i < cameras.length; i++) {
            inputs[i] = new VisionIOInputs();
        }
        fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    }
    
    @Override
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(inputs[i]);
            inputs[i].toSmartDashboard("Vision/Camera" + i);
        }
        
        if (enableVisionUpdates) {
            processVisionUpdates();
        }
        
        SmartDashboard.putBoolean("Vision/Enabled", enableVisionUpdates);
    }
    
    private void processVisionUpdates() {
        for (int i = 0; i < inputs.length; i++) {
            VisionIOInputs input = inputs[i];
            
            if (!input.hasTargets || input.estimatedPose == null) {
                continue;
            }
            
            Optional<VisionMeasurement> measurement = evaluateMeasurement(input, i);
            
            if (measurement.isPresent()) {
                addVisionMeasurement(measurement.get());
            }
        }
    }
    
    private Optional<VisionMeasurement> evaluateMeasurement(VisionIOInputs input, int cameraIndex) {
        if (input.poseAmbiguity > MAX_POSE_AMBIGUITY) {
            SmartDashboard.putBoolean("Vision/Camera" + cameraIndex + "/RejectedByAmbiguity", true);
            return Optional.empty();
        }
        
        if (input.averageTagDistance > MAX_TAG_DISTANCE_METERS) {
            SmartDashboard.putBoolean("Vision/Camera" + cameraIndex + "/RejectedByDistance", true);
            return Optional.empty();
        }
        
        if (input.averageTagArea < MIN_TAG_AREA) {
            SmartDashboard.putBoolean("Vision/Camera" + cameraIndex + "/RejectedByArea", true);
            return Optional.empty();
        }
        
        Matrix<N3, N1> stdDevs = calculateStdDevs(input.tagCount, input.averageTagDistance);
        
        VisionMeasurement measurement = new VisionMeasurement(
            input.estimatedPose, input.timestamp, stdDevs, input.tagCount
        );
        
        SmartDashboard.putBoolean("Vision/Camera" + cameraIndex + "/AcceptedMeasurement", true);
        return Optional.of(measurement);
    }
    
    private Matrix<N3, N1> calculateStdDevs(int tagCount, double avgDistance) {
        double baseStdDev = tagCount >= 2 ? MULTI_TAG_STD_DEV_BASE : SINGLE_TAG_STD_DEV_BASE;
        double distanceMultiplier = 1.0 + (avgDistance * avgDistance);
        
        double xyStdDev = baseStdDev * distanceMultiplier;
        double thetaStdDev = xyStdDev * 2.0;
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
    
    private void addVisionMeasurement(VisionMeasurement measurement) {
        SmartDashboard.putNumber("Vision/LatestMeasurement/X", measurement.pose.getX());
        SmartDashboard.putNumber("Vision/LatestMeasurement/Y", measurement.pose.getY());
        SmartDashboard.putNumber("Vision/LatestMeasurement/Rot", measurement.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/MeasurementTimestamp", measurement.timestamp);
        SmartDashboard.putNumber("Vision/TagCount", measurement.tagCount);
        
        
    }
    
    public void setVisionEnabled(boolean enabled) {
        this.enableVisionUpdates = enabled;
        SmartDashboard.putBoolean("Vision/Enabled", enabled);
    }
    
    public boolean isVisionEnabled() {
        return enableVisionUpdates;
    }
    
    public int getBestCameraIndex() {
        int bestIndex = 0;
        int maxScore = 0;
        
        for (int i = 0; i < inputs.length; i++) {
            if (!inputs[i].hasTargets) continue;
            
            int score = inputs[i].tagCount * 10 - (int)(inputs[i].averageTagDistance);
            
            if (score > maxScore) {
                maxScore = score;
                bestIndex = i;
            }
        }
        
        return bestIndex;
    }
    
    public static class VisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3, N1> stdDevs;
        public final int tagCount;
        
        public VisionMeasurement(Pose2d pose, double timestamp, 
                               Matrix<N3, N1> stdDevs, int tagCount) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDevs = stdDevs;
            this.tagCount = tagCount;
        }
    }
}