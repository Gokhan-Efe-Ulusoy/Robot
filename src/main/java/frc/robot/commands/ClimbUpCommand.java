
package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsytem;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class ClimbUpCommand extends CommandBase {



  private final IntakeSubsytem intake;



  public ClimbUpCommand(IntakeSubsytem intake) {
    
    this.intake=intake;
    addRequirements(intake);

  }

  
  @Override
  public void initialize() {
    
    
  }


  @Override
  public void execute() {
    intake.intakeAlgea();
  }


  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
