package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * IO interface for vision cameras
 * Supports multiple camera types (Limelight, PhotonVision, etc.)
 */
public interface VisionIO {
    
    /**
     * Input data from vision camera
     */
    class VisionIOInputs {
        public boolean connected = false;
        public boolean hasTargets = false;
        
        // Pose estimation
        public Pose2d estimatedPose = null;
        public double timestamp = 0.0;
        public double poseAmbiguity = 1.0;
        
        // Target information
        public int tagCount = 0;
        public int[] visibleTagIds = new int[0];
        public double averageTagDistance = 0.0;
        public double averageTagArea = 0.0;
        
        // Camera performance
        public double latencyMs = 0.0;
        public double fps = 0.0;
        public int pipelineIndex = 0;
    }
    
    /**
     * Update inputs from camera
     * @param inputs Input object to populate
     */
    default void updateInputs(VisionIOInputs inputs) {}
    
    /**
     * Set camera pipeline
     * @param pipeline Pipeline index to use
     */
    default void setPipeline(int pipeline) {}
    
    /**
     * Set LED mode
     * @param mode LED mode to use
     */
    default void setLEDMode(LEDMode mode) {}
    
    /**
     * LED mode options
     */
    enum LEDMode {
        PIPELINE(0),  // Use pipeline default
        OFF(1),       // Force off
        BLINK(2),     // Blink
        ON(3);        // Force on
        
        public final int value;
        
        LEDMode(int value) {
            this.value = value;
        }
    }
}