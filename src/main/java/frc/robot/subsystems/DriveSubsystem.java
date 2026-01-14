package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {

    private final Pigeon2 gyro;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Rotation2d gyroOffset = new Rotation2d();
    private boolean gyroHealthy = true;


    public DriveSubsystem() {

        gyro = new Pigeon2(DriveConstants.PIGEON_ID);

        frontLeft = new SwerveModule(1, 2, DriveConstants.FL_OFFSET);
        frontRight = new SwerveModule(3, 4, DriveConstants.FR_OFFSET);
        backLeft = new SwerveModule(5, 6, DriveConstants.BL_OFFSET);
        backRight = new SwerveModule(7, 8, DriveConstants.BR_OFFSET);

        kinematics = new SwerveDriveKinematics(
                new Translation2d(
                        DriveConstants.WHEEL_BASE / 2.0,
                        DriveConstants.TRACK_WIDTH / 2.0
                ),
                new Translation2d(
                        DriveConstants.WHEEL_BASE / 2.0,
                        -DriveConstants.TRACK_WIDTH / 2.0
                ),
                new Translation2d(
                        -DriveConstants.WHEEL_BASE / 2.0,
                        DriveConstants.TRACK_WIDTH / 2.0
                ),
                new Translation2d(
                        -DriveConstants.WHEEL_BASE / 2.0,
                        -DriveConstants.TRACK_WIDTH / 2.0
                )
        );

        odometry = new SwerveDriveOdometry(
                kinematics,
                getHeading(),
                getModulePositions()
        );

        zeroGyro();
    }

    @Override
    public void periodic() {
        gyroHealthy = gyro.getYaw().getStatus().isOK() && gyro.isConnected();
    
        if (gyroHealthy) {
            odometry.update(
                    getHeading(),
                    getModulePositions()
            );
        }
    }

    public void drive(
            double xSpeed,
            double ySpeed,
            double rot,
            boolean fieldRelative
    ) {

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rot,
                        getHeading()
                )
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        SwerveModuleState[] states =
                kinematics.toSwerveModuleStates(speeds);

        // Assuming m/s
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                DriveConstants.MAX_SPEED_MPS
        );

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public Rotation2d getHeading() {
        if (!gyroHealthy) {
            return new Rotation2d(); // fallback assuming zero
        }
    
        return Rotation2d.fromDegrees(
                gyro.getYaw().getValue()
        ).minus(gyroOffset);
    }

    public void zeroGyro() {
        gyroOffset = Rotation2d.fromDegrees(
                gyro.getYaw().getValue()
        );
    }
    public boolean isGyroHealthy() {
        return gyroHealthy;
    }
    

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getHeading(),
                getModulePositions(),
                pose
        );
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }
    public void setBrakeMode(boolean enable) {
        frontLeft.setBrakeMode(enable);
        frontRight.setBrakeMode(enable);
        backLeft.setBrakeMode(enable);
        backRight.setBrakeMode(enable);
    }
    
}
