package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
    
    private final NetworkTable limelightTable;
    private final String cameraName;
    private final Pose2d robotToCameraTransform;
    
    /**
     * @param cameraName    
     * @param robotToCameraTransform
     */
    public VisionIOLimelight(String cameraName, Pose2d robotToCameraTransform) {
        this.cameraName = cameraName;
        this.robotToCameraTransform = robotToCameraTransform;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(cameraName);
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = limelightTable.getEntry("tv").exists();
        if (!inputs.connected) {
            return;
        }
        
        double tv = limelightTable.getEntry("tv").getDouble(0);
        inputs.hasTargets = tv > 0.5;
        
        if (!inputs.hasTargets) {
            return;
        }
        
        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
        
        if (botpose.length >= 11 && botpose[7] > 0) {
            inputs.estimatedPose = new Pose2d(
                botpose[0],
                botpose[1],
                Rotation2d.fromDegrees(botpose[5])
            );
            
            double latencySeconds = botpose[6] / 1000.0;
            inputs.timestamp = (System.currentTimeMillis() / 1000.0) - latencySeconds;
            
            inputs.tagCount = (int) botpose[7];
            inputs.averageTagDistance = botpose[9];
            inputs.averageTagArea = botpose[10];
            
            inputs.visibleTagIds = getVisibleTagIds();
            
            inputs.poseAmbiguity = calculatePoseAmbiguity(botpose[8], botpose[9]);
            
            inputs.latencyMs = botpose[6];
            inputs.fps = limelightTable.getEntry("fps").getDouble(0.0);
        }
    }
    
    private int[] getVisibleTagIds() {
        return new int[0];
    }
    
    private double calculatePoseAmbiguity(double tagSpan, double avgDistance) {
        if (avgDistance < 0.1) {
            return 1.0;
        }
        
        double ambiguity = avgDistance / (tagSpan + 1.0);
        
        return Math.min(1.0, ambiguity / 10.0);
    }
    
    @Override
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    @Override
    public void setLEDMode(LEDMode mode) {
        int modeValue = switch (mode) {
            case PIPELINE -> 0;
            case OFF -> 1;
            case BLINK -> 2;
            case ON -> 3;
        };
        
        limelightTable.getEntry("ledMode").setNumber(modeValue);
    }
    
    public void setStreamMode(int mode) {
        limelightTable.getEntry("stream").setNumber(mode);
    }
    
    public void takeSnapshot() {
        limelightTable.getEntry("snapshot").setNumber(1);
    }
}