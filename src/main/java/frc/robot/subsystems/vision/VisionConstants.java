package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    
    public static final Pose2d FRONT_CAMERA_POSITION = new Pose2d(
        new Translation2d(
            Units.inchesToMeters(10.0),  
            Units.inchesToMeters(0.0)     
        ),
        Rotation2d.fromDegrees(0.0)     
    );
    
    public static final Pose2d BACK_CAMERA_POSITION = new Pose2d(
        new Translation2d(
            Units.inchesToMeters(-10.0),  
            Units.inchesToMeters(0.0)     
        ),
        Rotation2d.fromDegrees(180.0)  
    );
    
    public static final String FRONT_CAMERA_NAME = "limelight-front";
    
    public static final String BACK_CAMERA_NAME = "limelight-back";
    
    public static final int DEFAULT_PIPELINE = 0;
    
    public static final int APRILTAG_PIPELINE = 0;
    
    public static final int CORAL_PIPELINE = 1;
    
    public static final int ALGAE_PIPELINE = 2;
        
    public static final double MAX_POSE_AMBIGUITY = 0.25;
    
    public static final double MAX_TAG_DISTANCE_METERS = 4.5;
    
    public static final double MIN_TAG_AREA = 0.08;
    
    public static final int MIN_MULTI_TAG_COUNT = 2;
        
    public static final double SINGLE_TAG_STD_DEV_BASE = 0.9;
    
    public static final double MULTI_TAG_STD_DEV_BASE = 0.3;
    
    public static final double THETA_STD_DEV_MULTIPLIER = 2.5;
    
    public static final VisionIO.LEDMode TELEOP_LED_MODE = VisionIO.LEDMode.OFF;
    
    public static final VisionIO.LEDMode AUTO_LED_MODE = VisionIO.LEDMode.ON;
    
    public static final VisionIO.LEDMode DISABLED_LED_MODE = VisionIO.LEDMode.OFF;
        
    public static final boolean ENABLE_VISION_IN_AUTO = true;
    
    public static final boolean ENABLE_VISION_IN_TELEOP = true;
    
    public static final boolean USE_DYNAMIC_STD_DEV = true;
        
    public static final double FIELD_LENGTH_METERS = 16.54;
    
    public static final double FIELD_WIDTH_METERS = 8.21;
    
    
    public static class TagIDs {
        public static final int BLUE_REEF_LEFT = 11;
        public static final int BLUE_REEF_CENTER = 12;
        public static final int BLUE_REEF_RIGHT = 13;
        public static final int BLUE_PROCESSOR = 14;
        public static final int BLUE_BARGE_LEFT = 15;
        public static final int BLUE_BARGE_RIGHT = 16;
        
        public static final int RED_BARGE_LEFT = 1;
        public static final int RED_BARGE_RIGHT = 2;
        public static final int RED_PROCESSOR = 3;
        public static final int RED_REEF_LEFT = 4;
        public static final int RED_REEF_CENTER = 5;
        public static final int RED_REEF_RIGHT = 6;
    }
     
    public static final double CORAL_DETECTION_AREA = 0.5;
    
    public static final double ALGAE_DETECTION_AREA = 0.3;
    
    public static final double MAX_GAME_PIECE_ANGLE_ERROR = 10.0;
}