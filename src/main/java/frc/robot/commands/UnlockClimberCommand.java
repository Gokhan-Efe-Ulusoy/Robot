
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsytem;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class UnlockClimberCommand extends InstantCommand {



  private final ClimberSubsytem climber;


  public UnlockClimberCommand(ClimberSubsytem climber) {
    
    this.climber=climber;
    addRequirements(climber);

  }

  
  @Override
  public void initialize() {
    
    
  }


  @Override
  public void execute() {
    climber.climbDown();
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
