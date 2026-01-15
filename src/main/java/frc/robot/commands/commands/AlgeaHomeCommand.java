
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;


public class AlgeaHomeCommand extends InstantCommand {


  private final Superstructure superstructure;
  private final Intake intake;



  public HomeCommand(Superstructure superstructure, Intake intake) {
    this.superstructure=superstructure;
    this.intake=intake;
    addRequirements(superstructure, intake);

  }

  
  @Override
  public void initialize() {
    intake.stop();
    superstructure.goHome();
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
