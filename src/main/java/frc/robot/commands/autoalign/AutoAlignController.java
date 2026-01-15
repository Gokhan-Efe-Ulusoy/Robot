package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoAlignController {

    private final ProfiledPIDController x, y, theta;

    public AutoAlignController() {
        x = new ProfiledPIDController(
                AutoAlignConstants.X_KP, 0,
                AutoAlignConstants.X_KD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_SPEED,
                        AutoAlignConstants.MAX_ACCEL));

        y = new ProfiledPIDController(
                AutoAlignConstants.Y_KP, 0,
                AutoAlignConstants.Y_KD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_SPEED,
                        AutoAlignConstants.MAX_ACCEL));

        theta = new ProfiledPIDController(
                AutoAlignConstants.THETA_KP, 0,
                AutoAlignConstants.THETA_KD,
                new TrapezoidProfile.Constraints(
                        AutoAlignConstants.MAX_OMEGA,
                        AutoAlignConstants.MAX_ALPHA));

        theta.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d current, Pose2d target) {
        x.setGoal(target.getX());
        y.setGoal(target.getY());
        theta.setGoal(target.getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                x.calculate(current.getX()),
                y.calculate(current.getY()),
                theta.calculate(current.getRotation().getRadians()),
                current.getRotation());
    }

    public boolean atGoal() {
        return x.atGoal() && y.atGoal() && theta.atGoal();
    }

    public void reset(Pose2d pose) {
        x.reset(pose.getX());
        y.reset(pose.getY());
        theta.reset(pose.getRotation().getRadians());
    }
}
