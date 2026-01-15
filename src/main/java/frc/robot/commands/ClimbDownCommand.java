
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsytem;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class ClimbDownCommand extends CommandBase {



  private final ClimberSubsytem climber;


  public ClimbDownCommand(ClimberSubsytem climber) {
    
    this.climber=climber;
    addRequirements(climber);

  }

  
  @Override
  public void initialize() {
    
    
  }


  @Override
  public void execute() {
    climber.climbUp();
  }


  @Override
  public void end(boolean interrupted) {
    climber.hold();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
