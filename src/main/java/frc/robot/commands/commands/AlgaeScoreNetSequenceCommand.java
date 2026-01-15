package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;


public class AlgaeScoreNetSequenceCommand extends SequentialCommandGroup {

  public AlgaeScoreNetSequenceCommand(Superstructure superstructure, Intake intake) {

    addCommands(
      new NetScorePositionCommand(superstructure),
      new OuttakeCommand(intake).withTimeout(0.4),
      new AlgeaHomeCommand(superstructure)
    );
  }
}

