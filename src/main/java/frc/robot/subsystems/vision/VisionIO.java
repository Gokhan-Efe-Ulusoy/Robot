package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface VisionIO {
    
    public static class VisionIOInputs {
        public boolean connected = false;
        public boolean hasTargets = false;
        
        public Pose2d estimatedPose = null;
        public double timestamp = 0.0;
        public double poseAmbiguity = 1.0;
        
        public int tagCount = 0;
        public int[] visibleTagIds = new int[0];
        public double averageTagDistance = 0.0;
        public double averageTagArea = 0.0;
        
        public double latencyMs = 0.0;
        public double fps = 0.0;
        
        public void toSmartDashboard(String prefix) {
            SmartDashboard.putBoolean(prefix + "/Connected", connected);
            SmartDashboard.putBoolean(prefix + "/HasTargets", hasTargets);
            SmartDashboard.putNumber(prefix + "/TagCount", tagCount);
            SmartDashboard.putNumber(prefix + "/AvgDistance", averageTagDistance);
            SmartDashboard.putNumber(prefix + "/AvgArea", averageTagArea);
            SmartDashboard.putNumber(prefix + "/Ambiguity", poseAmbiguity);
            SmartDashboard.putNumber(prefix + "/LatencyMs", latencyMs);
            SmartDashboard.putNumber(prefix + "/FPS", fps);
            
            if (estimatedPose != null) {
                SmartDashboard.putNumber(prefix + "/PoseX", estimatedPose.getX());
                SmartDashboard.putNumber(prefix + "/PoseY", estimatedPose.getY());
                SmartDashboard.putNumber(prefix + "/PoseRot", estimatedPose.getRotation().getDegrees());
            }
        }
    }
    
    public default void updateInputs(VisionIOInputs inputs) {}
    public default void setPipeline(int pipeline) {}
    public default void setLEDMode(LEDMode mode) {}
    
    public enum LEDMode {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }
}