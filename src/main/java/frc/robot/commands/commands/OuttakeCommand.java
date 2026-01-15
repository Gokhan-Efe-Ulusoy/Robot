
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class OuttakeCommand extends CommandBase {



  private final Intake intake;



  public OuttakeCommand(Intake intake) {
    
    this.intake=intake;
    addRequirements(intake);

  }

  
  @Override
  public void initialize() {
    
    
  }


  @Override
  public void execute() {
    intake.intakeCoral();
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
