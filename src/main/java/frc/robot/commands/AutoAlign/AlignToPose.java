package frc.robot.commands.autoaligns;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Belirli field pozisyonuna hizalanma - Team 1323 Yaklaşımı
 * 
 * WPILib's HolonomicDriveController kullanır (best practice)
 * Profiled rotation control ile smooth movement
 * 
 * Team 1323'ün championship-winning approach'u:
 * Clean, maintainable, WPILib-integrated code.
 */
public class AlignToPose extends Command {
    
    private final Drive drive;
    private final Pose2d targetPose;
    private final HolonomicDriveController controller;
    
    // Tolerances
    private static final double POSITION_TOLERANCE_METERS = 0.03;
    private static final double ANGLE_TOLERANCE_DEGREES = 1.5;
    
    // Timeout
    private static final double ALIGNMENT_TIMEOUT_SECONDS = 3.0;
    private double startTime;
    
    /**
     * @param drive Drive subsystem
     * @param targetPose Hedef pozisyon (field-relative)
     */
    public AlignToPose(Drive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        
        // HolonomicDriveController setup (1323 style)
        this.controller = new HolonomicDriveController(
            // X and Y controllers
            new PIDController(2.5, 0.0, 0.05),
            new PIDController(2.5, 0.0, 0.05),
            
            // Theta controller with motion profiling
            new ProfiledPIDController(
                3.0, 0.0, 0.1,
                new TrapezoidProfile.Constraints(
                    Math.PI,        // Max angular velocity (rad/s)
                    2 * Math.PI     // Max angular acceleration (rad/s²)
                )
            )
        );
        
        // Set tolerances
        controller.setTolerance(new Pose2d(
            POSITION_TOLERANCE_METERS,
            POSITION_TOLERANCE_METERS,
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(ANGLE_TOLERANCE_DEGREES)
        ));
        
        // Enable theta wrapping
        controller.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        SmartDashboard.putString("AutoAlign/Status", 
            String.format("Aligning to (%.2f, %.2f, %.1f°)", 
                targetPose.getX(), 
                targetPose.getY(), 
                targetPose.getRotation().getDegrees()));
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        
        // HolonomicDriveController calculates optimal chassis speeds
        ChassisSpeeds targetSpeeds = controller.calculate(
            currentPose,
            targetPose,
            0.0,  // Desired linear velocity at target
            targetPose.getRotation()
        );
        
        // Convert to field-relative and drive
        drive.drive(
            targetSpeeds.vxMetersPerSecond,
            targetSpeeds.vyMetersPerSecond,
            targetSpeeds.omegaRadiansPerSecond,
            true  // Field-relative
        );
        
        // Logging
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double thetaError = targetPose.getRotation()
            .minus(currentPose.getRotation())
            .getRadians();
        
        SmartDashboard.putNumber("AutoAlign/XError", xError);
        SmartDashboard.putNumber("AutoAlign/YError", yError);
        SmartDashboard.putNumber("AutoAlign/ThetaError", Math.toDegrees(thetaError));
        SmartDashboard.putNumber("AutoAlign/Distance", 
            Math.hypot(xError, yError));
        SmartDashboard.putBoolean("AutoAlign/AtGoal", controller.atReference());
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
        
        if (interrupted) {
            SmartDashboard.putString("AutoAlign/Status", "Interrupted");
        } else {
            SmartDashboard.putString("AutoAlign/Status", "Aligned!");
        }
    }
    
    @Override
    public boolean isFinished() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        // Timeout check
        if (currentTime - startTime > ALIGNMENT_TIMEOUT_SECONDS) {
            SmartDashboard.putString("AutoAlign/Status", "Timeout!");
            return true;
        }
        
        // Check if at reference using controller's built-in method
        return controller.atReference();
    }
}