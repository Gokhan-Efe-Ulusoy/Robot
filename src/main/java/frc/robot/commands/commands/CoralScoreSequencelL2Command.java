package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;


public class CoralScoreSequencelL2Command extends SequentialCommandGroup {

  public CoralScoreSequencelL1Command(Superstructure superstructure, Intake intake) {

    addCommands(
      new L1CoralScorePositionCommand(superstructure),

      new OuttakeCommand(intake).withTimeout(0.4),

      new L1SplitScoreCoralPositionCommand(superstructure)
    );
  }
}

