
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;


public class AlgeaFloorPickupPositionCommand extends InstantCommand {


  private final Superstructure superstructure;




  public AlgeaHomeCommand(Superstructure superstructure, Intake intake) {
    this.superstructure=superstructure;

    addRequirements(superstructure);

  }

  
  @Override
  public void initialize() {
    
    superstructure.goToAlgaeHome();
  }


  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
