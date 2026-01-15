package frc.robot.commands.autoalign;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TeleopAutoAlignCommand extends Command {

    private final DriveSubsystem drive;
    private final Vision vision;
    private final AutoAlignController controller = new AutoAlignController();

    public TeleopAutoAlignCommand(
            DriveSubsystem drive,
            Vision vision
    ) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller.reset(drive.getPose());
    }

    @Override
    public void execute() {
        Optional<Pose2d> target =
                ReefTargetSelector.selectBestReefTarget(
                        drive.getPose(),
                        vision.getVisibleTags()
                );

        if (target.isEmpty()) {
            // Vision yok → robotu zorla sürme
            drive.drive(new ChassisSpeeds());
            return;
        }

        drive.drive(
                controller.calculate(
                        drive.getPose(),
                        target.get()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Teleop’ta driver bırakana kadar
    }

    @Override
    public void end(boolean interrupted) {
        // stop() yok, sıfır hız var
        drive.drive(new ChassisSpeeds());
    }
}
