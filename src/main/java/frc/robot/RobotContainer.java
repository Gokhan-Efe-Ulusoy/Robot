package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign.TeleopAutoAlignCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.HoldClimberCommand;
import frc.robot.commands.UnlockClimberCommand;
import frc.robot.commands.commands.AlgaeScoreNetSequenceCommand;
import frc.robot.commands.commands.AlgaeScoreProcessorSequenceCommand;
import frc.robot.commands.commands.CoralScoreSequenceL1Command;
import frc.robot.commands.commands.CoralScoreSequenceL2Command;
import frc.robot.commands.commands.CoralScoreSequenceL3Command;
import frc.robot.commands.commands.HoldGamepieceCommand;
import frc.robot.commands.commands.IntakeCoralCommand;
import frc.robot.commands.commands.OuttakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralCradleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StraightenatorSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final GripperArmSubsystem m_gripperArmSubsystem = new GripperArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final StraightenatorSubsystem m_straightenatorSubsystem = new StraightenatorSubsystem();
  private final CoralCradleSubsystem m_coralCradleSubsystem = new CoralCradleSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  
  private final VisionIOLimelight m_frontLimelight = new VisionIOLimelight("limelight-front");
  private final VisionIOLimelight m_backLimelight = new VisionIOLimelight("limelight-back");
  private final Vision m_vision = new Vision(m_driveSubsystem, m_frontLimelight, m_backLimelight);
  
  private final SuperstructureSubsystem m_superstructureSubsystem = new SuperstructureSubsystem(
      m_elevatorSubsystem,
      m_gripperArmSubsystem
  );

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(1);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    configureAutonomous();
    initializeSubsystems();
  }

  private void configureDefaultCommands() {
    m_driveSubsystem.setDefaultCommand(
        Commands.run(() -> {
          double xSpeed = -m_driverController.getLeftY();
          double ySpeed = -m_driverController.getLeftX();
          double rot = -m_driverController.getRightX();
          
          xSpeed = applyDeadbandAndSquare(xSpeed);
          ySpeed = applyDeadbandAndSquare(ySpeed);
          rot = applyDeadbandAndSquare(rot);
          
          m_driveSubsystem.drive(xSpeed, ySpeed, rot, true);
        }, m_driveSubsystem)
    );
    
    m_intakeSubsystem.setDefaultCommand(
        Commands.run(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );
    
    m_climberSubsystem.setDefaultCommand(
        Commands.run(() -> m_climberSubsystem.hold(), m_climberSubsystem)
    );
  }

  private void configureBindings() {
    m_driverController.leftTrigger(0.3)
        .whileTrue(new TeleopAutoAlignCommand(m_driveSubsystem, m_vision));
    
    m_driverController.rightTrigger(0.3)
        .whileTrue(Commands.run(() -> {
          double currentHeading = m_driveSubsystem.getHeading().getDegrees();
          double snappedHeading = Math.round(currentHeading / 90.0) * 90.0;
          double rotOutput = (snappedHeading - currentHeading) * 0.05;
          m_driveSubsystem.drive(0, 0, rotOutput, true);
        }, m_driveSubsystem));
    
    m_driverController.a()
        .onTrue(Commands.runOnce(() -> m_driveSubsystem.zeroGyro()));
    
    m_driverController.b()
        .onTrue(Commands.runOnce(() -> 
            m_driveSubsystem.setBrakeMode(!m_driveSubsystem.getBrakeMode())));
    
    m_driverController.x()
        .whileTrue(Commands.parallel(
            new IntakeCoralCommand(m_intakeSubsystem),
            Commands.run(() -> m_straightenatorSubsystem.runIn())
        ));
    
    m_driverController.y()
        .whileTrue(new OuttakeCommand(m_intakeSubsystem));
    
    new Trigger(() -> 
        m_driverController.getStartButton() && m_driverController.getBackButton())
        .onTrue(Commands.runOnce(() -> {
          m_intakeSubsystem.stop();
          m_straightenatorSubsystem.stop();
          m_gripperArmSubsystem.stopGripper();
        }));
    
    m_operatorController.a()
        .onTrue(new CoralScoreSequenceL1Command(m_superstructureSubsystem, m_intakeSubsystem));
    
    m_operatorController.b()
        .onTrue(new CoralScoreSequenceL2Command(m_superstructureSubsystem, m_intakeSubsystem));
    
    m_operatorController.x()
        .onTrue(new CoralScoreSequenceL3Command(m_superstructureSubsystem, m_intakeSubsystem));
    
    m_operatorController.y()
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> m_superstructureSubsystem.goToL4CoralScore()),
            Commands.waitUntil(() -> m_superstructureSubsystem.atTarget()),
            new OuttakeCommand(m_intakeSubsystem).withTimeout(0.4),
            Commands.runOnce(() -> m_superstructureSubsystem.goToL4CoralLower())
        ));
    
    m_operatorController.leftBumper()
        .onTrue(new AlgaeScoreNetSequenceCommand(m_superstructureSubsystem, m_intakeSubsystem));
    
    m_operatorController.rightBumper()
        .onTrue(new AlgaeScoreProcessorSequenceCommand(m_superstructureSubsystem, m_intakeSubsystem));
    
    m_operatorController.leftTrigger(0.3)
        .whileTrue(Commands.parallel(
            Commands.runOnce(() -> m_superstructureSubsystem.goToCoralPickup()),
            new IntakeCoralCommand(m_intakeSubsystem),
            Commands.run(() -> m_straightenatorSubsystem.runIn())
        ));
    
    m_operatorController.rightTrigger(0.3)
        .whileTrue(Commands.parallel(
            Commands.runOnce(() -> m_superstructureSubsystem.goToAlgaeFloorPickup()),
            Commands.run(() -> m_intakeSubsystem.intakeAlgae())
        ));
    
    m_operatorController.back()
        .onTrue(Commands.runOnce(() -> m_superstructureSubsystem.goHome()));
    
    m_operatorController.start()
        .whileTrue(new HoldGamepieceCommand(m_intakeSubsystem));
    
    new Trigger(() -> 
        m_operatorController.getStartButton() && m_operatorController.getBackButton())
        .onTrue(new UnlockClimberCommand(m_climberSubsystem));
    
    new Trigger(() -> 
        m_operatorController.getPOV() == 0 && 
        m_operatorController.getStartButton())
        .whileTrue(new ClimbUpCommand(m_intakeSubsystem));
    
    new Trigger(() -> 
        m_operatorController.getPOV() == 180 && 
        m_operatorController.getStartButton())
        .whileTrue(new ClimbDownCommand(m_climberSubsystem));
    
    new Trigger(() -> 
        m_operatorController.getPOV() == -1 && 
        m_operatorController.getStartButton())
        .onTrue(new HoldClimberCommand(m_climberSubsystem));
    
    Trigger coralDetected = new Trigger(() -> 
        m_vision.getVisibleTags().stream()
            .anyMatch(tag -> tag.getId() >= 17 && tag.getId() <= 24) &&
        m_operatorController.leftTrigger().getAsBoolean());
    
    coralDetected.whileTrue(
        Commands.run(() -> {
          double adjustX = m_vision.getVisibleTags().get(0).getFieldPose().getX() - 
                          m_driveSubsystem.getPose().getX();
          double adjustY = m_vision.getVisibleTags().get(0).getFieldPose().getY() - 
                          m_driveSubsystem.getPose().getY();
          m_driveSubsystem.drive(adjustX * 0.5, adjustY * 0.5, 0, true);
        }, m_driveSubsystem)
    );
  }

  private void configureAutonomous() {
    m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Score L1 + Mobility", 
        Commands.sequence(
            new CoralScoreSequenceL1Command(m_superstructureSubsystem, m_intakeSubsystem),
            Commands.run(() -> m_driveSubsystem.drive(1.5, 0, 0, true)).withTimeout(2.0)
        ));
    m_autoChooser.addOption("Score L2 + Mobility",
        Commands.sequence(
            new CoralScoreSequenceL2Command(m_superstructureSubsystem, m_intakeSubsystem),
            Commands.run(() -> m_driveSubsystem.drive(1.5, 0, 0, true)).withTimeout(2.0)
        ));
    m_autoChooser.addOption("Two Coral + Mobility",
        Commands.sequence(
            new CoralScoreSequenceL1Command(m_superstructureSubsystem, m_intakeSubsystem),
            Commands.run(() -> m_driveSubsystem.drive(-2.0, 0, 0, true)).withTimeout(2.0),
            Commands.runOnce(() -> m_superstructureSubsystem.goToCoralPickup()),
            new IntakeCoralCommand(m_intakeSubsystem).withTimeout(1.0),
            Commands.run(() -> m_driveSubsystem.drive(2.0, 0, 0, true)).withTimeout(2.0),
            new CoralScoreSequenceL2Command(m_superstructureSubsystem, m_intakeSubsystem)
        ));
    
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void initializeSubsystems() {
    m_elevatorSubsystem.home();
    m_gripperArmSubsystem.setArmPosition(GripperArmSubsystem.ArmPosition.STOW);
    m_coralCradleSubsystem.close();
    m_frontLimelight.setPipeline(0);
    m_backLimelight.setPipeline(0);
    m_frontLimelight.setLEDMode(VisionIOLimelight.LEDMode.OFF);
    m_backLimelight.setLEDMode(VisionIOLimelight.LEDMode.OFF);
    m_driveSubsystem.zeroGyro();
    m_climberSubsystem.hold();
  }

  private double applyDeadbandAndSquare(double value) {
    if (Math.abs(value) < 0.1) {
      return 0.0;
    }
    return Math.copySign(value * value, value);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
  
  public boolean isVisionAssistActive() {
    return m_driverController.leftTrigger().getAsBoolean();
  }
  
  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }
  
  public Vision getVision() {
    return m_vision;
  }
}
