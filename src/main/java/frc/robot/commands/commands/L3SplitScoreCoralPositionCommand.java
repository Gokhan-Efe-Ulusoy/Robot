
package frc.robot.commands;

import frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;



public class L3SplitScoreCoralPositionCommand extends InstantCommand {



  private final Supertructure superstructure;



  public L3SplitScoreCoralPositionCommand (Superstructure superstructure) {
    
    this.superstructure=superstructure;
    addRequirements(superstructure);

  }

  
  @Override
  public void initialize() {
    superstructure.goToL3CoralLower();
  }


  @Override
  public void execute() {
   
  }


  @Override
  public void end(boolean interrupted) {
  }
    


  @Override
  public boolean isFinished() {
    return false;
  }
}
