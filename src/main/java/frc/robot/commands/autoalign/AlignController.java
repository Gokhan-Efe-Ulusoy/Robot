package frc.robot.commands.autoalign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AlignController {

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public AlignController() {
        xController = new PIDController(3.2, 0.0, 0.25);
        yController = new PIDController(3.2, 0.0, 0.25);
        thetaController = new PIDController(5.5, 0.0, 0.35);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        thetaController.setTolerance(Math.toRadians(1.5));
    }

    public ChassisSpeeds calculate(
            Pose2d currentPose,
            Pose2d targetPose,
            double maxVel,
            double maxOmega
    ) {
        double xSpeed =
                xController.calculate(
                        currentPose.getX(),
                        targetPose.getX()
                );

        double ySpeed =
                yController.calculate(
                        currentPose.getY(),
                        targetPose.getY()
                );

        double omega =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()
                );

        xSpeed = MathUtil.clamp(xSpeed, -maxVel, maxVel);
        ySpeed = MathUtil.clamp(ySpeed, -maxVel, maxVel);
        omega  = MathUtil.clamp(omega, -maxOmega, maxOmega);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                omega,
                currentPose.getRotation()
        );
    }

    public boolean atGoal() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && thetaController.atSetpoint();
    }

    public void reset() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }
}
