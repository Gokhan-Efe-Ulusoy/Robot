package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class AutoAlignController {

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private AutoAlignState state = AutoAlignState.SEEK;
    private Pose2d targetPose;
    private final AutoAlignMode mode;

    public AutoAlignController(AutoAlignMode mode) {
        this.mode = mode;

        xController = new ProfiledPIDController(
                AutoAlignConstants.TRANSLATION_kP,
                AutoAlignConstants.TRANSLATION_kI,
                AutoAlignConstants.TRANSLATION_kD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_TRANSLATION_VEL,
                        AutoAlignConstants.MAX_TRANSLATION_ACCEL
                )
        );

        yController = new ProfiledPIDController(
                AutoAlignConstants.TRANSLATION_kP,
                AutoAlignConstants.TRANSLATION_kI,
                AutoAlignConstants.TRANSLATION_kD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_TRANSLATION_VEL,
                        AutoAlignConstants.MAX_TRANSLATION_ACCEL
                )
        );

        thetaController = new ProfiledPIDController(
                AutoAlignConstants.ROTATION_kP,
                AutoAlignConstants.ROTATION_kI,
                AutoAlignConstants.ROTATION_kD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_ROTATION_VEL,
                        AutoAlignConstants.MAX_ROTATION_ACCEL
                )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset(Pose2d currentPose) {
        state = AutoAlignState.SEEK;
        targetPose = null;

        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
    }

    public ChassisSpeeds update(
            Pose2d currentPose,
            Pose2d visionTargetPose,
            VisionConfidence confidence
    ) {
        double now = Timer.getFPGATimestamp();

        double maxAge = mode == AutoAlignMode.AUTONOMOUS
                ? AutoAlignConstants.AUTO_MAX_TARGET_AGE
                : AutoAlignConstants.TELEOP_MAX_TARGET_AGE;

        boolean visionValid =
                confidence != null && confidence.isUsable(now, maxAge);

        switch (state) {
            case SEEK -> {
                if (visionValid) {
                    targetPose = visionTargetPose;
                    state = AutoAlignState.APPROACH;
                }
            }

            case APPROACH -> {
                if (visionValid) {
                    targetPose = visionTargetPose;
                } else if (mode == AutoAlignMode.AUTONOMOUS) {
                    state = AutoAlignState.SEEK;
                    targetPose = null;
                    return new ChassisSpeeds();
                }

                if (targetPose != null &&
                        currentPose.getTranslation()
                                .getDistance(targetPose.getTranslation())
                                < AutoAlignConstants.APPROACH_DISTANCE) {
                    state = AutoAlignState.FINAL_ALIGN;
                }
            }

            case FINAL_ALIGN -> {
                if (visionValid) {
                    targetPose = visionTargetPose;
                } else if (mode == AutoAlignMode.AUTONOMOUS) {
                    state = AutoAlignState.SEEK;
                    targetPose = null;
                    return new ChassisSpeeds();
                }
            }
        }

        if (targetPose == null) {
            return new ChassisSpeeds();
        }

        double vx = xController.calculate(
                currentPose.getX(),
                targetPose.getX()
        );
        double vy = yController.calculate(
                currentPose.getY(),
                targetPose.getY()
        );
        double omega = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, currentPose.getRotation()
        );
    }

    public boolean isAligned(Pose2d currentPose) {
        if (targetPose == null) return false;

        Translation2d error =
                currentPose.getTranslation()
                        .minus(targetPose.getTranslation());

        double rotError =
                Math.abs(
                        currentPose.getRotation()
                                .minus(targetPose.getRotation())
                                .getRadians()
                );

        return error.getNorm() < AutoAlignConstants.POSITION_TOLERANCE
                && rotError < AutoAlignConstants.ROTATION_TOLERANCE;
    }
}
