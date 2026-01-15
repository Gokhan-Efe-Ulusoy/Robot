package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;


public interface VisionIO {
    
    
    class VisionIOInputs {
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
        public int pipelineIndex = 0;
    }
    
    /**
     * @param inputs
     */
    default void updateInputs(VisionIOInputs inputs) {}
    
    /**
     * @param pipeline
     */
    default void setPipeline(int pipeline) {}
    
    /**
     * @param mode
     */
    default void setLEDMode(LEDMode mode) {}
    
    enum LEDMode {
        PIPELINE(0),  
        OFF(1),       
        BLINK(2),     
        ON(3);        
        
        public final int value;
        
        LEDMode(int value) {
            this.value = value;
        }
    }
}