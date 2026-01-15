package frc.robot.commands.autoalign;

import edu.wpi.first.wpilibj2.command.Command;
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
    public void execute() {
        vision.getBestReefTarget(drive.getPose())
                .ifPresent(target ->
                        drive.drive(
                                controller.calculate(
                                        drive.getPose(), target)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
