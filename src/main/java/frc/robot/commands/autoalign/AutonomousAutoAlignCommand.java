package frc.robot.commands.autoalign;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutonomousAutoAlignCommand extends Command {

    private final DriveSubsystem drive;
    private final Vision vision;
    private final AutoAlignController controller = new AutoAlignController();

    private AutoAlignState state;
    private double startTime;

    public AutonomousAutoAlignCommand(
            DriveSubsystem drive,
            Vision vision
    ) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        state = AutoAlignState.SEEKING;
        startTime = Timer.getFPGATimestamp();
        controller.reset(drive.getPose());
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        if (now - startTime > AutoAlignConstants.AUTON_TIMEOUT_SEC) {
            state = AutoAlignState.FAILED;
            return;
        }

        vision.getBestReefTarget(drive.getPose()).ifPresentOrElse(
                target -> {
                    state = AutoAlignState.ALIGNING;
                    drive.drive(controller.calculate(drive.getPose(), target));
                },
                () -> state = AutoAlignState.BLIND_ALIGN
        );

        if (state == AutoAlignState.ALIGNING && controller.atGoal()) {
            state = AutoAlignState.LOCKED;
        }
    }

    @Override
    public boolean isFinished() {
        return state == AutoAlignState.LOCKED
            || state == AutoAlignState.FAILED;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
