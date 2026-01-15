package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.ArrayList;
import java.util.List;

/**
 * Limelight camera implementation for VisionIO
 * Supports Limelight 2, 2+, 3, and 3G
 */
public class VisionIOLimelight implements VisionIO {
    
    private final NetworkTable limelightTable;
    private final String cameraName;
    private final ObjectMapper jsonMapper;
    
    private long lastSuccessfulUpdate = 0;
    private static final long CONNECTION_TIMEOUT_MS = 1000;
    
    /**
     * Creates a new Limelight vision IO
     * @param cameraName NetworkTable name of the Limelight (e.g., "limelight-front")
     */
    public VisionIOLimelight(String cameraName) {
        this.cameraName = cameraName;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(cameraName);
        this.jsonMapper = new ObjectMapper();
        
        // Set default pipeline and LED mode
        setPipeline(VisionConstants.DEFAULT_PIPELINE);
        setLEDMode(LEDMode.PIPELINE);
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Check connection
        boolean hasRecentData = (System.currentTimeMillis() - lastSuccessfulUpdate) < CONNECTION_TIMEOUT_MS;
        inputs.connected = limelightTable.getEntry("tv").exists() && hasRecentData;
        
        if (!inputs.connected) {
            inputs.hasTargets = false;
            return;
        }
        
        // Check if target is visible
        double tv = limelightTable.getEntry("tv").getDouble(0);
        inputs.hasTargets = tv > 0.5;
        
        if (!inputs.hasTargets) {
            inputs.estimatedPose = null;
            inputs.tagCount = 0;
            inputs.visibleTagIds = new int[0];
            return;
        }
        
        // Get botpose from Limelight (MegaTag2 output)
        // Format: [x, y, z, roll, pitch, yaw] in meters/degrees
        double[] botpose = limelightTable.getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);
        
        if (botpose.length >= 6) {
            inputs.estimatedPose = new Pose2d(
                botpose[0],  // x in meters
                botpose[1],  // y in meters
                Rotation2d.fromDegrees(botpose[5])  // yaw in degrees
            );
        } else {
            inputs.estimatedPose = null;
        }
        
        // Get latency
        double tl = limelightTable.getEntry("tl").getDouble(0);  // Pipeline latency
        double cl = limelightTable.getEntry("cl").getDouble(0);  // Capture latency
        inputs.latencyMs = tl + cl;
        
        // Calculate timestamp (FPGA time - latency)
        inputs.timestamp = Timer.getFPGATimestamp() - (inputs.latencyMs / 1000.0);
        
        // Get FPS
        inputs.fps = limelightTable.getEntry("fps").getDouble(0);
        
        // Get pipeline index
        inputs.pipelineIndex = (int) limelightTable.getEntry("getpipe").getDouble(0);
        
        // Parse JSON for detailed tag information
        parseJsonData(inputs);
        
        // Calculate tag distance and area
        calculateTagMetrics(inputs);
        
        // Mark successful update
        lastSuccessfulUpdate = System.currentTimeMillis();
    }
    
    private void parseJsonData(VisionIOInputs inputs) {
        try {
            String jsonString = limelightTable.getEntry("json").getString("");
            if (jsonString.isEmpty()) {
                inputs.tagCount = 0;
                inputs.visibleTagIds = new int[0];
                return;
            }
            
            JsonNode root = jsonMapper.readTree(jsonString);
            JsonNode results = root.get("Results");
            
            if (results == null || !results.isArray()) {
                inputs.tagCount = 0;
                inputs.visibleTagIds = new int[0];
                return;
            }
            
            // Extract visible tag IDs
            List<Integer> tagIds = new ArrayList<>();
            for (JsonNode result : results) {
                if (result.has("fID")) {
                    tagIds.add(result.get("fID").asInt());
                }
            }
            
            inputs.tagCount = tagIds.size();
            inputs.visibleTagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
            
            // Get pose ambiguity from first result
            if (results.size() > 0 && results.get(0).has("ambiguity")) {
                inputs.poseAmbiguity = results.get(0).get("ambiguity").asDouble();
            } else {
                // Fallback: estimate ambiguity based on tag count and distance
                inputs.poseAmbiguity = estimatePoseAmbiguity(inputs.tagCount, inputs.averageTagDistance);
            }
            
        } catch (Exception e) {
            // JSON parsing failed, use fallback values
            inputs.tagCount = limelightTable.getEntry("tid").getDouble(-1) >= 0 ? 1 : 0;
            inputs.visibleTagIds = inputs.tagCount > 0 
                ? new int[]{(int) limelightTable.getEntry("tid").getDouble(0)} 
                : new int[0];
            inputs.poseAmbiguity = estimatePoseAmbiguity(inputs.tagCount, inputs.averageTagDistance);
        }
    }
    
    private void calculateTagMetrics(VisionIOInputs inputs) {
        // Get target area (percentage of image)
        double ta = limelightTable.getEntry("ta").getDouble(0);
        inputs.averageTagArea = ta / 100.0;  // Convert percentage to decimal
        
        // Get 3D distance to target
        double[] camtran = limelightTable.getEntry("camtran").getDoubleArray(new double[6]);
        if (camtran.length >= 3) {
            // Calculate 3D distance from camera translation
            inputs.averageTagDistance = Math.sqrt(
                camtran[0] * camtran[0] + 
                camtran[1] * camtran[1] + 
                camtran[2] * camtran[2]
            );
        } else {
            // Fallback: estimate from area
            if (inputs.averageTagArea > 0) {
                // Rough estimation: larger area = closer distance
                inputs.averageTagDistance = Math.sqrt(0.5 / inputs.averageTagArea);
            } else {
                inputs.averageTagDistance = 0;
            }
        }
    }
    
    private double estimatePoseAmbiguity(int tagCount, double avgDistance) {
        // Multi-tag is more reliable
        if (tagCount >= 2) {
            return 0.05;  // Very low ambiguity
        }
        
        // Single tag - ambiguity increases with distance
        if (avgDistance < 1.0) return 0.10;
        if (avgDistance < 2.0) return 0.15;
        if (avgDistance < 3.0) return 0.20;
        return 0.30;
    }
    
    @Override
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    @Override
    public void setLEDMode(LEDMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.value);
    }
    
    /**
     * Set camera mode (vision processor vs driver camera)
     * @param visionMode true for vision processing, false for driver camera
     */
    public void setCameraMode(boolean visionMode) {
        limelightTable.getEntry("camMode").setNumber(visionMode ? 0 : 1);
    }
    
    /**
     * Set stream mode for driver station
     * @param mode 0=Standard, 1=PiP Main, 2=PiP Secondary
     */
    public void setStreamMode(int mode) {
        limelightTable.getEntry("stream").setNumber(mode);
    }
    
    /**
     * Take a snapshot
     */
    public void takeSnapshot() {
        limelightTable.getEntry("snapshot").setNumber(1);
    }
    
    /**
     * Get the camera name
     */
    public String getCameraName() {
        return cameraName;
    }
}