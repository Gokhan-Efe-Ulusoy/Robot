
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;


public class L2AlgeaPickupPositionCommand extends InstantCommand {


  private final Superstructure superstructure;




  public L2AlgeaPickupPositionCommand(Superstructure superstructure, Intake intake) {
    this.superstructure=superstructure;

    addRequirements(superstructure);

  }

  
  @Override
  public void initialize() {
    
    superstructure.goToAlgaeFloorPickup();
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
