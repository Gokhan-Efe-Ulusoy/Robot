package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
    
    private final String cameraName;
    private final Supplier<Pose2d> robotPoseSupplier;
    private boolean simulateTargets = false;
    
    /**
     * @param cameraName 
     * @param robotPoseSupplier
     */
    public VisionIOSim(String cameraName, Supplier<Pose2d> robotPoseSupplier) {
        this.cameraName = cameraName;
        this.robotPoseSupplier = robotPoseSupplier;
        System.out.println("[VisionIOSim] " + cameraName + " initialized (mock mode)");
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.hasTargets = simulateTargets;
        inputs.latencyMs = 35.0; 
        inputs.timestamp = System.currentTimeMillis() / 1000.0;
        inputs.fps = 90.0; 
        if (simulateTargets) {
            inputs.estimatedPose = robotPoseSupplier.get();
            inputs.tagCount = 1;
            inputs.visibleTagIds = new int[]{1}; 
            inputs.averageTagDistance = 2.0;
            inputs.averageTagArea = 0.5;
            inputs.poseAmbiguity = 0.05; 
        } else {
            inputs.estimatedPose = null;
            inputs.tagCount = 0;
            inputs.visibleTagIds = new int[0];
            inputs.averageTagDistance = 0.0;
            inputs.averageTagArea = 0.0;
            inputs.poseAmbiguity = 1.0;
        }
    }
    
    @Override
    public void setPipeline(int pipeline) {
        System.out.println("[VisionIOSim] " + cameraName + " pipeline set to " + pipeline);
    }
    
    @Override
    public void setLEDMode(LEDMode mode) {
        System.out.println("[VisionIOSim] " + cameraName + " LED mode set to " + mode);
    }
    
    public void setSimulateTargets(boolean simulate) {
        this.simulateTargets = simulate;
    }
}