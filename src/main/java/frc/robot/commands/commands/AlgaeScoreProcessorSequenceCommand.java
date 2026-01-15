package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;


public class AlgaeScoreProcessorSequenceCommand extends SequentialCommandGroup {

  public AlgaeScoreProcessorSequenceCommand(Superstructure superstructure, Intake intake) {

    addCommands(
      new ProcessorScorePositionCommand(superstructure),
      new OuttakeCommand(intake).withTimeout(0.4),
      new AlgeaHomeCommand(superstructure)
    );
  }
}

