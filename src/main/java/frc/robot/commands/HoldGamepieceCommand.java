
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class HoldGamepieceCommand extends CommandBase {



  private final Intake intake;



  public HoldGamepieceCommand(Intake intake) {
    
    this.intake=intake;
    addRequirements(intake);

  }

  
  @Override
  public void initialize() {
  
    
  }


  @Override
  public void execute() {
    intake.outtake();
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
