package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;


public class CoralScoreSequencelL3Command extends SequentialCommandGroup {

  public CoralScoreSequencelL3Command(Superstructure superstructure, Intake intake) {

    addCommands(
      new L3CoralScorePositionCommand(superstructure),

      new OuttakeCommand(intake).withTimeout(0.4),

      new L3SplitScoreCoralPositionCommand(superstructure)
    );
  }
}

