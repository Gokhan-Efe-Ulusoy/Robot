package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignCommand extends Command {

    private final DriveSubsystem drive;
    private final Vision vision;

    private Pose2d targetPose;
    private AutoAlignState state = AutoAlignState.IDLE;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    public AutoAlignCommand(
            DriveSubsystem drive,
            Vision vision
    ) {
        this.drive = drive;
        this.vision = vision;

        xController = new ProfiledPIDController(
                Align2025Constants.X_KP,
                0,
                Align2025Constants.X_KD,
                new TrapezoidProfile.Constraints(
                        Align2025Constants.MAX_SPEED,
                        Align2025Constants.MAX_ACCEL
                )
        );

        yController = new ProfiledPIDController(
                Align2025Constants.Y_KP,
                0,
                Align2025Constants.Y_KD,
                new TrapezoidProfile.Constraints(
                        Align2025Constants.MAX_SPEED,
                        Align2025Constants.MAX_ACCEL
                )
        );

        thetaController = new ProfiledPIDController(
                Align2025Constants.THETA_KP,
                0,
                Align2025Constants.THETA_KD,
                new TrapezoidProfile.Constraints(
                        Align2025Constants.MAX_OMEGA,
                        Align2025Constants.MAX_ALPHA
                )
        );

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        state = AutoAlignState.SEEK_TARGET;
        targetPose = null;
        xController.reset(drive.getPose().getX());
        yController.reset(drive.getPose().getY());
        thetaController.reset(drive.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        switch (state) {

            case IDLE -> drive.drive(0, 0, 0, true);

            case SEEK_TARGET -> {
                vision.getBestReefTarget(robotPose).ifPresentOrElse(
                        pose -> {
                            targetPose = pose;
                            state = AutoAlignState.APPROACH;
                        },
                        () -> drive.drive(0, 0, 0, true)
                );
            }

            case APPROACH -> {
                runAlign(robotPose, 0.6, 0.6);

                if (robotPose.getTranslation()
                        .getDistance(targetPose.getTranslation()) < 0.6) {
                    state = AutoAlignState.FINAL_ALIGN;
                }
            }

            case FINAL_ALIGN -> {
                runAlign(robotPose, 0.35, 0.4);

                if (xController.atGoal()
                        && yController.atGoal()
                        && thetaController.atGoal()) {
                    state = AutoAlignState.LOCKED;
                }
            }

            case LOCKED -> drive.drive(0, 0, 0, true);
        }
    }

    private void runAlign(Pose2d robotPose, double maxSpeed, double maxOmega) {

        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());

        double xOut = xController.calculate(robotPose.getX());
        double yOut = yController.calculate(robotPose.getY());
        double thetaOut = thetaController.calculate(
                robotPose.getRotation().getRadians()
        );

        xOut = clamp(xOut, maxSpeed);
        yOut = clamp(yOut, maxSpeed);
        thetaOut = clamp(thetaOut, maxOmega);

        ChassisSpeeds driver = drive.getDriverSpeeds();

        drive.drive(
                xOut + driver.vxMetersPerSecond * 0.25,
                yOut + driver.vyMetersPerSecond * 0.25,
                thetaOut + driver.omegaRadiansPerSecond * 0.25,
                true
        );
    }

    private double clamp(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }

    @Override
    public boolean isFinished() {
        return state == AutoAlignState.LOCKED;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
    }
}
