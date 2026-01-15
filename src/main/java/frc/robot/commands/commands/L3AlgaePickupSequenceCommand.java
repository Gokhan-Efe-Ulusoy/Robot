package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;


public class L3AlgaePickupSequenceCommand extends SequentialCommandGroup {

  public L3AlgaePickupSequenceCommand(Superstructure superstructure, Intake intake) {

    addCommands(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(intake::hasAlgae).withTimeout(2.0),
            new L2AlgeaPickupPositionCommand(superstructure),
            new IntakeAlgaeCommand(intake)
        ),

        new HoldGamepieceCommand(intake),

        new AlgaeHomeCommand(superstructure)
    );
  }
}

