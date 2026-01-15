
package frc.robot.commands;

import frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;



public class L4SplitScoreCoralPositionCommand extends InstantCommand {



  private final Supertructure superstructure;



  public L4SplitScoreCoralPositionCommand (Superstructure superstructure) {
    
    this.superstructure=superstructure;
    addRequirements(superstructure);

  }

  
  @Override
  public void initialize() {
    superstructure.goToL4CoralLower();
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
