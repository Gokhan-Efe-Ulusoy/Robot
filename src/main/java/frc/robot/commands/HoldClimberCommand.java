
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsytem;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class HoldClimberCommand extends InstantCommand {



  private final ClimberSubsytem climber;


  public HoldClimberCommand(ClimberSubsytem climber) {
    
    this.climber=climber;
    addRequirements(climber);

  }

  
  @Override
  public void initialize() {
  
    
  }


  @Override
  public void execute() {
      climber.hold();
  }


  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
