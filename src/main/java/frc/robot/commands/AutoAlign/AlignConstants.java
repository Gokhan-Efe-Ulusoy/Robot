package frc.robot.commands.autoaligns;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Auto align için revize edilmiş sabitler.
 * 
 * Top teams'in best practices'ine göre organize edilmiş:
 * - Team 2910: Simple, effective constants
 * - Team 254: Distance-based tuning
 * - Team 1323: WPILib conventions
 */
public class AlignConstants {
    
    // ============================================================================
    // PID TUNING - Team 2910 World Champion Values
    // ============================================================================
    
    public static final class PIDGains {
        // X/Y Translation - Conservative for smooth motion
        public static final double X_KP = 2.5;
        public static final double X_KI = 0.0;   // Minimal I to avoid windup
        public static final double X_KD = 0.05;  // Small D for damping
        
        public static final double Y_KP = 2.5;
        public static final double Y_KI = 0.0;
        public static final double Y_KD = 0.05;
        
        // Rotation - Slightly more aggressive
        public static final double THETA_KP = 3.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.1;
        
        // Motion Profile Constraints (for HolonomicDriveController)
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = 2 * Math.PI;
    }
    
    // ============================================================================
    // TOLERANCES - Tighter than before (2910 + 254 best practices)
    // ============================================================================
    
    public static final double POSITION_TOLERANCE_METERS = 0.03;  // 3 cm (was 5 cm)
    public static final double ANGLE_TOLERANCE_DEGREES = 1.5;     // 1.5° (was 2°)
    
    // Stability - Must be within tolerance for this duration
    public static final double STABILITY_TIME_SECONDS = 0.2;
    
    // ============================================================================
    // SPEED LIMITS
    // ============================================================================
    
    public static final double MAX_SPEED_MPS = 2.5;  // Slightly increased
    public static final double MAX_ROTATION_RAD_PER_SEC = Math.PI;
    
    // ============================================================================
    // VISION FILTERING - Team 254 Approach
    // ============================================================================
    
    /**
     * Maximum robot velocity for reliable vision updates
     * Above this speed, vision measurements are less reliable
     */
    public static final double MAX_VELOCITY_FOR_VISION_MPS = 1.5;
    
    /**
     * Maximum distance to trust single-tag measurements
     */
    public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 3.0;
    
    /**
     * Maximum pose ambiguity to accept vision measurement
     */
    public static final double MAX_POSE_AMBIGUITY = 0.2;
    
    // ============================================================================
    // TIMEOUT
    // ============================================================================
    
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 3.0;
    
    // ============================================================================
    // APRILTAG OFFSETS - Game-specific positioning
    // ============================================================================
    
    /**
     * Amp scoring offset - 1m in front of tag
     */
    public static final Transform2d AMP_OFFSET = new Transform2d(
        new Translation2d(1.0, 0.0),
        new Rotation2d()
    );
    
    /**
     * Speaker scoring offset - 2m in front of tag
     * (adjust based on shooter range)
     */
    public static final Transform2d SPEAKER_OFFSET = new Transform2d(
        new Translation2d(2.0, 0.0),
        new Rotation2d()
    );
    
    /**
     * Source/HP station offset - 0.8m for intake
     */
    public static final Transform2d SOURCE_OFFSET = new Transform2d(
        new Translation2d(0.8, 0.0),
        new Rotation2d()
    );
    
    /**
     * Close-range precise offset (for climbing, etc.)
     */
    public static final Transform2d PRECISE_OFFSET = new Transform2d(
        new Translation2d(0.5, 0.0),
        new Rotation2d()
    );
    
    // ============================================================================
    // APRILTAG IDs - 2025 REEFSCAPE FIELD LAYOUT
    // ============================================================================
    
    public static final class TagIDs {
        // Blue Alliance (tags 1-6)
        public static final int BLUE_SOURCE_RIGHT = 1;
        public static final int BLUE_SOURCE_LEFT = 2;
        public static final int BLUE_SPEAKER_OFFSET = 3;
        public static final int BLUE_SPEAKER_CENTER = 4;
        public static final int BLUE_AMP = 6;
        
        // Red Alliance (tags 5, 7-10)
        public static final int RED_AMP = 5;
        public static final int RED_SPEAKER_OFFSET = 7;
        public static final int RED_SPEAKER_CENTER = 8;
        public static final int RED_SOURCE_LEFT = 9;
        public static final int RED_SOURCE_RIGHT = 10;
    }
    
    // ============================================================================
    // FIELD COORDINATES - 2025 Reefscape (16.54m x 8.21m)
    // ============================================================================
    
    public static final class FieldCoordinates {
        // Field dimensions
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.21;
        
        // Blue alliance preset positions
        public static final Pose2d BLUE_AMP_SCORE = new Pose2d(
            14.02, 7.99, Rotation2d.fromDegrees(-90)
        );
        
        public static final Pose2d BLUE_SPEAKER_SCORE = new Pose2d(
            15.51, 5.96, Rotation2d.fromDegrees(180)
        );
        
        public static final Pose2d BLUE_SOURCE_INTAKE = new Pose2d(
            15.08, 0.66, Rotation2d.fromDegrees(120)
        );
        
        // Red alliance preset positions
        public static final Pose2d RED_AMP_SCORE = new Pose2d(
            1.52, 7.99, Rotation2d.fromDegrees(-90)
        );
        
        public static final Pose2d RED_SPEAKER_SCORE = new Pose2d(
            1.03, 5.96, Rotation2d.fromDegrees(0)
        );
        
        public static final Pose2d RED_SOURCE_INTAKE = new Pose2d(
            1.46, 0.66, Rotation2d.fromDegrees(60)
        );
    }
    
    // ============================================================================
    // DYNAMIC TUNING - Distance-based adjustments (254 approach)
    // ============================================================================
    
    /**
     * Calculate speed scaling based on distance to target
     * Closer = slower for precision
     * 
     * @param distanceMeters Distance to target
     * @return Speed multiplier (0.0 to 1.0)
     */
    public static double getSpeedScaling(double distanceMeters) {
        if (distanceMeters < 0.5) {
            // Very close - slow down significantly
            return 0.3 + (distanceMeters / 0.5) * 0.4;  // 30-70% speed
        } else if (distanceMeters < 2.0) {
            // Medium distance - moderate speed
            return 0.7 + ((distanceMeters - 0.5) / 1.5) * 0.3;  // 70-100% speed
        } else {
            // Far - full speed
            return 1.0;
        }
    }
    
    /**
     * Get PID multiplier based on alliance and game state
     * Can be used for different auto vs teleop tuning
     * 
     * @param isAuto True if in autonomous
     * @return PID multiplier
     */
    public static double getPIDMultiplier(boolean isAuto) {
        // In auto, be more aggressive (faster cycle time)
        // In teleop, be more conservative (smoother for driver)
        return isAuto ? 1.2 : 1.0;
    }
}