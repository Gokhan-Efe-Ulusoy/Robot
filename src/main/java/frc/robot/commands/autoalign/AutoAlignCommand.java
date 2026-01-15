package frc.robot.commands.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignCommand extends Command {

    private final DriveSubsystem drive;
    private final Vision vision;
    private final AlignController controller;

    private Pose2d targetPose;

    public AutoAlignCommand(
            DriveSubsystem drive,
            Vision vision,
            Pose2d targetPose) {

        this.drive = drive;
        this.vision = vision;
        this.targetPose = targetPose;
        this.controller = new AlignController();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();

        var speeds = controller.calculate(currentPose, targetPose);

        drive.drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }
}
