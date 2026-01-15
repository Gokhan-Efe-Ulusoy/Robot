package frc.robot.commands.autoalign;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autoalign.*;

public class TeleopAutoAlignCommand extends Command {

    private final AutoAlignController controller;

    public TeleopAutoAlignCommand(AutoAlignController controller) {
        this.controller = controller;
    }

    @Override
    public boolean isFinished() {
        return false; // sürücü bırakır
    }
}
