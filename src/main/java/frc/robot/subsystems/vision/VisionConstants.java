package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for vision system - FRC 2025 Reefscape
 */
public class VisionConstants {
    
    // === Camera Configuration ===
    
    /** Front camera position relative to robot center */
    public static final Pose2d FRONT_CAMERA_POSITION = new Pose2d(
        new Translation2d(
            Units.inchesToMeters(10.0),   // 10 inches forward
            Units.inchesToMeters(0.0)     // Centered left-right
        ),
        Rotation2d.fromDegrees(0.0)       // Facing forward
    );
    
    /** Back camera position relative to robot center */
    public static final Pose2d BACK_CAMERA_POSITION = new Pose2d(
        new Translation2d(
            Units.inchesToMeters(-10.0),  // 10 inches back
            Units.inchesToMeters(0.0)     // Centered left-right
        ),
        Rotation2d.fromDegrees(180.0)     // Facing backward
    );
    
    /** Front camera NetworkTable name */
    public static final String FRONT_CAMERA_NAME = "limelight-front";
    
    /** Back camera NetworkTable name */
    public static final String BACK_CAMERA_NAME = "limelight-back";
    
    /** Default pipeline index */
    public static final int DEFAULT_PIPELINE = 0;
    
    /** AprilTag detection pipeline */
    public static final int APRILTAG_PIPELINE = 0;
    
    /** Coral detection pipeline (if using ML) */
    public static final int CORAL_PIPELINE = 1;
    
    /** Algae detection pipeline (if using ML) */
    public static final int ALGAE_PIPELINE = 2;
    
    // === Vision Filtering Thresholds ===
    
    /** Maximum acceptable pose ambiguity (0.0-1.0) */
    public static final double MAX_POSE_AMBIGUITY = 0.25;
    
    /** Maximum distance to trust a single tag (meters) */
    public static final double MAX_TAG_DISTANCE_METERS = 4.5;
    
    /** Minimum tag area to accept measurement (0.0-1.0) */
    public static final double MIN_TAG_AREA = 0.08;
    
    /** Minimum number of tags for multi-tag mode */
    public static final int MIN_MULTI_TAG_COUNT = 2;
    
    // === Standard Deviation Calculation ===
    
    /** Base standard deviation for single-tag measurements (meters) */
    public static final double SINGLE_TAG_STD_DEV_BASE = 0.9;
    
    /** Base standard deviation for multi-tag measurements (meters) */
    public static final double MULTI_TAG_STD_DEV_BASE = 0.3;
    
    /** Multiplier for theta standard deviation */
    public static final double THETA_STD_DEV_MULTIPLIER = 2.5;
    
    // === LED Modes ===
    
    /** LED mode during teleop */
    public static final VisionIO.LEDMode TELEOP_LED_MODE = VisionIO.LEDMode.OFF;
    
    /** LED mode during autonomous */
    public static final VisionIO.LEDMode AUTO_LED_MODE = VisionIO.LEDMode.ON;
    
    /** LED mode during disabled */
    public static final VisionIO.LEDMode DISABLED_LED_MODE = VisionIO.LEDMode.OFF;
    
    // === Vision Enable Flags ===
    
    /** Enable vision updates in autonomous */
    public static final boolean ENABLE_VISION_IN_AUTO = true;
    
    /** Enable vision updates in teleop */
    public static final boolean ENABLE_VISION_IN_TELEOP = true;
    
    /** Use dynamic standard deviation based on distance */
    public static final boolean USE_DYNAMIC_STD_DEV = true;
    
    // === Field Dimensions (2025 Reefscape) ===
    
    /** Field length in meters */
    public static final double FIELD_LENGTH_METERS = 16.54;
    
    /** Field width in meters */
    public static final double FIELD_WIDTH_METERS = 8.21;
    
    // === AprilTag IDs (2025 Reefscape) ===
    
    public static class TagIDs {
        // Blue Alliance
        public static final int BLUE_REEF_LEFT = 11;
        public static final int BLUE_REEF_CENTER = 12;
        public static final int BLUE_REEF_RIGHT = 13;
        public static final int BLUE_PROCESSOR = 14;
        public static final int BLUE_BARGE_LEFT = 15;
        public static final int BLUE_BARGE_RIGHT = 16;
        
        // Red Alliance
        public static final int RED_BARGE_LEFT = 1;
        public static final int RED_BARGE_RIGHT = 2;
        public static final int RED_PROCESSOR = 3;
        public static final int RED_REEF_LEFT = 4;
        public static final int RED_REEF_CENTER = 5;
        public static final int RED_REEF_RIGHT = 6;
    }
    
    // === Game Piece Detection Thresholds ===
    
    /** Minimum area for coral detection (percentage) */
    public static final double CORAL_DETECTION_AREA = 0.5;
    
    /** Minimum area for algae detection (percentage) */
    public static final double ALGAE_DETECTION_AREA = 0.3;
    
    /** Maximum angle offset for game piece alignment (degrees) */
    public static final double MAX_GAME_PIECE_ANGLE_ERROR = 10.0;
}