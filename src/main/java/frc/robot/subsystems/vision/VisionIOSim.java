package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
public class VisionIOSim implements VisionIO {
    
    private final String cameraName;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<Pose2d> knownTagPoses;
    
    private boolean simulateTargets = true;
    private double simulatedLatency = 35.0;  
    private double simulatedNoise = 0.02;    
    private int currentPipeline = 0;
    private LEDMode currentLEDMode = LEDMode.PIPELINE;
    
    private static final double MAX_VISION_RANGE = 4.5; 
    private static final double CAMERA_FOV = Math.toRadians(90);  
    
    /**
     * @param cameraName
     * @param robotPoseSupplier 
     * @param knownTagPoses 
     */
    public VisionIOSim(String cameraName, Supplier<Pose2d> robotPoseSupplier, List<Pose2d> knownTagPoses) {
        this.cameraName = cameraName;
        this.robotPoseSupplier = robotPoseSupplier;
        this.knownTagPoses = knownTagPoses != null ? knownTagPoses : new ArrayList<>();
        
        System.out.println("[VisionIOSim] " + cameraName + " initialized with " + 
                         this.knownTagPoses.size() + " known tags");
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.pipelineIndex = currentPipeline;
        
        inputs.latencyMs = simulatedLatency + (Math.random() * 10 - 5);
        inputs.timestamp = Timer.getFPGATimestamp() - (inputs.latencyMs / 1000.0);
        
        inputs.fps = 85.0 + (Math.random() * 10);
        
        if (!simulateTargets || knownTagPoses.isEmpty()) {
            inputs.hasTargets = false;
            inputs.estimatedPose = null;
            inputs.tagCount = 0;
            inputs.visibleTagIds = new int[0];
            inputs.averageTagDistance = 0;
            inputs.averageTagArea = 0;
            inputs.poseAmbiguity = 1.0;
            return;
        }
        
        Pose2d actualPose = robotPoseSupplier.get();
        List<TagDetection> visibleTags = findVisibleTags(actualPose);
        
        if (visibleTags.isEmpty()) {
            inputs.hasTargets = false;
            inputs.estimatedPose = null;
            inputs.tagCount = 0;
            inputs.visibleTagIds = new int[0];
            inputs.averageTagDistance = 0;
            inputs.averageTagArea = 0;
            inputs.poseAmbiguity = 1.0;
            return;
        }
        
        inputs.hasTargets = true;
        inputs.tagCount = visibleTags.size();
        inputs.visibleTagIds = visibleTags.stream()
            .mapToInt(t -> t.id)
            .toArray();
        
        double noisyX = actualPose.getX() + (Math.random() * 2 - 1) * simulatedNoise;
        double noisyY = actualPose.getY() + (Math.random() * 2 - 1) * simulatedNoise;
        double noisyTheta = actualPose.getRotation().getRadians() + 
                          (Math.random() * 2 - 1) * Math.toRadians(1.0);
        
        inputs.estimatedPose = new Pose2d(noisyX, noisyY, new Rotation2d(noisyTheta));
        
        double totalDistance = 0;
        double totalArea = 0;
        
        for (TagDetection tag : visibleTags) {
            totalDistance += tag.distance;
            totalArea += tag.area;
        }
        
        inputs.averageTagDistance = totalDistance / visibleTags.size();
        inputs.averageTagArea = totalArea / visibleTags.size();
        
        if (inputs.tagCount >= 2) {
            inputs.poseAmbiguity = 0.05 + (Math.random() * 0.03);
        } else {
            double baseAmbiguity = inputs.averageTagDistance < 2.0 ? 0.10 : 0.15;
            inputs.poseAmbiguity = baseAmbiguity + (Math.random() * 0.05);
        }
    }
    
    private List<TagDetection> findVisibleTags(Pose2d robotPose) {
        List<TagDetection> visible = new ArrayList<>();
        
        for (int i = 0; i < knownTagPoses.size(); i++) {
            Pose2d tagPose = knownTagPoses.get(i);
            
            double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            
            if (distance > MAX_VISION_RANGE) {
                continue;
            }
            
            Rotation2d angleToTag = new Rotation2d(
                tagPose.getX() - robotPose.getX(),
                tagPose.getY() - robotPose.getY()
            );
            
            double angleDiff = Math.abs(
                angleToTag.minus(robotPose.getRotation()).getRadians()
            );
            
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            angleDiff = Math.abs(angleDiff);
            
            if (angleDiff > CAMERA_FOV) {
                continue;
            }
            
            double area = Math.min(1.0, 0.5 / (distance * distance));
            
            // Add some randomness to area
            area *= (0.9 + Math.random() * 0.2);
            
            visible.add(new TagDetection(i + 1, distance, area));
        }
        
        return visible;
    }
    
    @Override
    public void setPipeline(int pipeline) {
        this.currentPipeline = pipeline;
        System.out.println("[VisionIOSim] " + cameraName + " pipeline set to " + pipeline);
    }
    
    @Override
    public void setLEDMode(LEDMode mode) {
        this.currentLEDMode = mode;
        System.out.println("[VisionIOSim] " + cameraName + " LED mode set to " + mode);
    }
    
    
    public void setSimulateTargets(boolean simulate) {
        this.simulateTargets = simulate;
    }
    
    public void setSimulatedLatency(double latencyMs) {
        this.simulatedLatency = latencyMs;
    }
    
    public void setSimulatedNoise(double noiseMeters) {
        this.simulatedNoise = noiseMeters;
    }
    private static class TagDetection {
        final int id;
        final double distance;
        final double area;
        
        TagDetection(int id, double distance, double area) {
            this.id = id;
            this.distance = distance;
            this.area = area;
        }
    }
}