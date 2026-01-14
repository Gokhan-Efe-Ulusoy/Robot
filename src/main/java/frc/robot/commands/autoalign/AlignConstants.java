package frc.robot.commands.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlignConstants {

    public static final class PIDGains {
        public static final double X_KP = 2.5;
        public static final double X_KI = 0.0;   
        public static final double X_KD = 0.05; 
        
        public static final double Y_KP = 2.5;
        public static final double Y_KI = 0.0;
        public static final double Y_KD = 0.05;
        
        public static final double THETA_KP = 3.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.1;
        
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = 2 * Math.PI;
    }
    
    
    public static final double POSITION_TOLERANCE_METERS = 0.03;  
    public static final double ANGLE_TOLERANCE_DEGREES = 1.5;     
    
    public static final double STABILITY_TIME_SECONDS = 0.2;
    public static final double MAX_SPEED_MPS = 2.5;  
    public static final double MAX_ROTATION_RAD_PER_SEC = Math.PI;
   
    public static final double MAX_VELOCITY_FOR_VISION_MPS = 1.5;
    
    public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 3.0;
    
    
    public static final double MAX_POSE_AMBIGUITY = 0.2;
    
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 3.0;
    
    public static final Transform2d AMP_OFFSET = new Transform2d(
        new Translation2d(1.0, 0.0),
        new Rotation2d()
    );
    
    public static final Transform2d SPEAKER_OFFSET = new Transform2d(
        new Translation2d(2.0, 0.0),
        new Rotation2d()
    );
    
    public static final Transform2d SOURCE_OFFSET = new Transform2d(
        new Translation2d(0.8, 0.0),
        new Rotation2d()
    );
    public static final Transform2d PRECISE_OFFSET = new Transform2d(
        new Translation2d(0.5, 0.0),
        new Rotation2d()
    );
    
    public static final class TagIDs {
        public static final int BLUE_SOURCE_RIGHT = 1;
        public static final int BLUE_SOURCE_LEFT = 2;
        public static final int BLUE_SPEAKER_OFFSET = 3;
        public static final int BLUE_SPEAKER_CENTER = 4;
        public static final int BLUE_AMP = 6;
        
        public static final int RED_AMP = 5;
        public static final int RED_SPEAKER_OFFSET = 7;
        public static final int RED_SPEAKER_CENTER = 8;
        public static final int RED_SOURCE_LEFT = 9;
        public static final int RED_SOURCE_RIGHT = 10;
    }
    
    
    public static final class FieldCoordinates {
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.21;
        
        public static final Pose2d BLUE_AMP_SCORE = new Pose2d(
            14.02, 7.99, Rotation2d.fromDegrees(-90)
        );
        
        public static final Pose2d BLUE_SPEAKER_SCORE = new Pose2d(
            15.51, 5.96, Rotation2d.fromDegrees(180)
        );
        
        public static final Pose2d BLUE_SOURCE_INTAKE = new Pose2d(
            15.08, 0.66, Rotation2d.fromDegrees(120)
        );
        
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
    
    /**
     * @param distanceMeters
     * @return
     */
    public static double getSpeedScaling(double distanceMeters) {
        if (distanceMeters < 0.5) {
            return 0.3 + (distanceMeters / 0.5) * 0.4; 
        } else if (distanceMeters < 2.0) {
            return 0.7 + ((distanceMeters - 0.5) / 1.5) * 0.3;  
        } else {
            return 1.0;
        }
    }
    
    /**
     * @param isAuto
     * @return
     */
    public static double getPIDMultiplier(boolean isAuto) {
        return isAuto ? 1.2 : 1.0;
    }
}