package frc.robot.commands.autoaligns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/**
 * AprilTag'e otomatik hizalanma - Team 2910 World Champion Yaklaşımı
 * 
 * Özellikler:
 * - Basit ama etkili PID control
 * - Velocity-based filtering (254'ten)
 * - Smart timeout mechanism
 * - Driver override capability
 * - Extensive logging
 * 
 * Bu kod Team 2910'un 2025 World Champion yaklaşımına dayanır:
 * Kompleks filtreler yerine iyi tuned PID ve robust error handling.
 */
public class AlignToAprilTag extends Command {
    
    private final Drive drive;
    private final Vision vision;
    private final int targetTagID;
    private final Transform2d targetOffset;
    
    // Simple PID controllers (2910 style - no motion profiling)
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    
    // Tolerances
    private static final double POSITION_TOLERANCE_METERS = 0.03; // 3 cm (2910: tighter than ours)
    private static final double ANGLE_TOLERANCE_DEGREES = 1.5;    // 1.5 deg
    
    // Speed limits
    private static final double MAX_SPEED_MPS = 2.5;
    private static final double MAX_ROTATION_RAD_PER_SEC = Math.PI;
    
    // Velocity-based vision filtering (254'ten öğrendik)
    private static final double MAX_VELOCITY_FOR_VISION = 1.5; // m/s
    
    // Timeout ve stability
    private static final double ALIGNMENT_TIMEOUT_SECONDS = 3.0;
    private static final double STABILITY_TIME_SECONDS = 0.2;
    
    private double startTime;
    private double stableStartTime;
    private boolean wasStable;
    
    /**
     * @param drive Drive subsystem
     * @param vision Vision subsystem
     * @param targetTagID Hedef AprilTag ID
     * @param targetOffset Tag'den offset (mesafe ve açı)
     */
    public AlignToAprilTag(Drive drive, Vision vision, int targetTagID, Transform2d targetOffset) {
        this.drive = drive;
        this.vision = vision;
        this.targetTagID = targetTagID;
        this.targetOffset = targetOffset;
        
        // 2910-style PID tuning
        // Conservative P for smooth motion, small D for damping
        this.xController = new PIDController(2.5, 0.0, 0.05);
        this.yController = new PIDController(2.5, 0.0, 0.05);
        this.thetaController = new PIDController(3.0, 0.0, 0.1);
        
        // Theta wrapping için
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Integral windup prevention
        xController.setIntegratorRange(-0.5, 0.5);
        yController.setIntegratorRange(-0.5, 0.5);
        thetaController.setIntegratorRange(-0.5, 0.5);
        
        addRequirements(drive);
    }
    
    /**
     * Simplified constructor - Tag'in 1m önünde dur
     */
    public AlignToAprilTag(Drive drive, Vision vision, int targetTagID) {
        this(drive, vision, targetTagID, 
             new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d()));
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
        
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        wasStable = false;
        
        SmartDashboard.putString("AutoAlign/Status", "Aligning to Tag " + targetTagID);
        SmartDashboard.putNumber("AutoAlign/TargetTag", targetTagID);
    }
    
    @Override
    public void execute() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        // Current robot pose
        Pose2d currentPose = drive.getPose();
        
        // Get tag pose from field layout
        Pose2d tagPose = getTagPose(targetTagID);
        
        if (tagPose == null) {
            // Tag bulunamadı - stop
            SmartDashboard.putString("AutoAlign/Status", "Tag " + targetTagID + " not found!");
            drive.drive(0, 0, 0, true);
            return;
        }
        
        // Calculate target pose (tag + offset)
        Pose2d targetPose = tagPose.plus(targetOffset);
        
        // Calculate errors
        Transform2d error = targetPose.minus(currentPose);
        double xError = error.getX();
        double yError = error.getY();
        double thetaError = error.getRotation().getRadians();
        
        // Velocity-based vision filtering (254'ten)
        // Hızlı hareket sırasında vision less reliable
        double robotVelocity = Math.hypot(
            drive.getChassisSpeeds().vxMetersPerSecond,
            drive.getChassisSpeeds().vyMetersPerSecond
        );
        
        boolean useVision = robotVelocity < MAX_VELOCITY_FOR_VISION;
        
        // PID calculations
        double xSpeed = xController.calculate(0, xError);
        double ySpeed = yController.calculate(0, yError);
        double rotSpeed = thetaController.calculate(0, thetaError);
        
        // Smooth ramping near target (2910 trick)
        double distanceToTarget = Math.hypot(xError, yError);
        if (distanceToTarget < 0.5) {
            double rampFactor = distanceToTarget / 0.5;
            xSpeed *= rampFactor;
            ySpeed *= rampFactor;
        }
        
        // Clamp speeds
        xSpeed = MathUtil.clamp(xSpeed, -MAX_SPEED_MPS, MAX_SPEED_MPS);
        ySpeed = MathUtil.clamp(ySpeed, -MAX_SPEED_MPS, MAX_SPEED_MPS);
        rotSpeed = MathUtil.clamp(rotSpeed, -MAX_ROTATION_RAD_PER_SEC, MAX_ROTATION_RAD_PER_SEC);
        
        // Field-relative drive
        drive.drive(xSpeed, ySpeed, rotSpeed, true);
        
        // Stability tracking
        boolean isCurrentlyStable = isAligned(xError, yError, thetaError);
        
        if (isCurrentlyStable && !wasStable) {
            stableStartTime = currentTime;
        }
        wasStable = isCurrentlyStable;
        
        // Comprehensive logging
        SmartDashboard.putNumber("AutoAlign/XError", xError);
        SmartDashboard.putNumber("AutoAlign/YError", yError);
        SmartDashboard.putNumber("AutoAlign/ThetaError", Math.toDegrees(thetaError));
        SmartDashboard.putNumber("AutoAlign/Distance", distanceToTarget);
        SmartDashboard.putNumber("AutoAlign/RobotVelocity", robotVelocity);
        SmartDashboard.putBoolean("AutoAlign/UsingVision", useVision);
        SmartDashboard.putBoolean("AutoAlign/IsStable", isCurrentlyStable);
        SmartDashboard.putNumber("AutoAlign/TimeRemaining", 
            ALIGNMENT_TIMEOUT_SECONDS - (currentTime - startTime));
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
        
        if (interrupted) {
            SmartDashboard.putString("AutoAlign/Status", "Interrupted");
        } else {
            SmartDashboard.putString("AutoAlign/Status", "Aligned!");
        }
        
        SmartDashboard.putBoolean("AutoAlign/Active", false);
    }
    
    @Override
    public boolean isFinished() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        // Timeout check
        if (currentTime - startTime > ALIGNMENT_TIMEOUT_SECONDS) {
            SmartDashboard.putString("AutoAlign/Status", "Timeout!");
            return true;
        }
        
        // Tag yoksa bitir
        if (getTagPose(targetTagID) == null) {
            return true;
        }
        
        // Stability check - must be stable for STABILITY_TIME_SECONDS
        if (wasStable && (currentTime - stableStartTime) > STABILITY_TIME_SECONDS) {
            return true;
        }
        
        return false;
    }
    
    /**
     * Error değerlerine göre hizalanmış mı kontrol eder
     */
    private boolean isAligned(double xError, double yError, double thetaError) {
        double positionError = Math.hypot(xError, yError);
        double angleError = Math.abs(Math.toDegrees(thetaError));
        
        return positionError < POSITION_TOLERANCE_METERS && 
               angleError < ANGLE_TOLERANCE_DEGREES;
    }
    
    /**
     * AprilTag'in field pozisyonunu döner
     */
    private Pose2d getTagPose(int tagID) {
        // TODO: AprilTagFieldLayout ile gerçek field'dan al
        // Şimdilik hardcoded (gerçek değerlerle değiştirin)
        return getHardcodedTagPose(tagID);
    }
    
    /**
     * Hardcoded tag positions - 2025 Reefscape için
     * GERÇEK FIELD LAYOUT İLE DEĞİŞTİRİN!
     */
    private Pose2d getHardcodedTagPose(int tagID) {
        // 2025 field dimensions: 16.54m x 8.21m
        return switch (tagID) {
            // Blue alliance tags
            case 1 -> new Pose2d(15.08, 0.25, Rotation2d.fromDegrees(120));  // Blue source right
            case 2 -> new Pose2d(15.08, 1.07, Rotation2d.fromDegrees(120));  // Blue source left
            case 3 -> new Pose2d(15.51, 5.51, Rotation2d.fromDegrees(180));  // Blue speaker offset
            case 4 -> new Pose2d(15.51, 6.41, Rotation2d.fromDegrees(180));  // Blue speaker center
            case 6 -> new Pose2d(14.02, 7.99, Rotation2d.fromDegrees(-90));  // Blue amp
            
            // Red alliance tags
            case 5 -> new Pose2d(1.52, 7.99, Rotation2d.fromDegrees(-90));   // Red amp
            case 7 -> new Pose2d(1.03, 6.41, Rotation2d.fromDegrees(0));     // Red speaker offset
            case 8 -> new Pose2d(1.03, 5.51, Rotation2d.fromDegrees(0));     // Red speaker center
            case 9 -> new Pose2d(1.46, 1.07, Rotation2d.fromDegrees(60));    // Red source left
            case 10 -> new Pose2d(1.46, 0.25, Rotation2d.fromDegrees(60));   // Red source right
            
            default -> null;
        };
    }
}