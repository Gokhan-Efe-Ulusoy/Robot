package frc.robot.commands.autoalign;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.autoalign.AutoAlignController;

public class AutonomousAutoAlignCommand extends Command {

    private final AutoAlignController controller;
    private final Pose2d poseSupplier;
    private final double timeoutSec;

    private double startTime;

    public AutonomousAutoAlignCommand(
            AutoAlignController controller,
            Pose2d poseSupplier,
            double timeoutSec
    ) {
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.timeoutSec = timeoutSec;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return controller.isAligned(poseSupplier)
                || Timer.getFPGATimestamp() - startTime > timeoutSec;
    }
}
