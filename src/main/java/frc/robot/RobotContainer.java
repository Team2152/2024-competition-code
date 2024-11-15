package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteTracking;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.controls.DualDriver;
import frc.robot.controls.SingleDriver;

public class RobotContainer {
  // public final Limelight m_frontCamera;
  // public final Limelight m_rearCamera;
  public final NoteTracking m_noteTracking;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;

  private final DualDriver m_dualDriving;
  private final SingleDriver m_singleDriving;

  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  public final CommandXboxController m_operatorController;

  public final LEDs m_leds;

  // public final Orchestra m_music;

  
  public RobotContainer() {
    //m_rearCamera = new Limelight(Constants.Vision.kRearCameraName, Constants.Vision.kRearRobotToCam);
    m_noteTracking = new NoteTracking();
    // m_frontCamera = new Limelight();

    m_drivetrain = new Drivetrain();
    m_intake = new Intake();
    m_shooter = new Shooter();

    m_leds = new LEDs(OIConstants.kLedPort, OIConstants.kLedLength);

    NamedCommands.registerCommand("DeployIntake", m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut));
    NamedCommands.registerCommand("RetractIntake", m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn));

    NamedCommands.registerCommand("StartIntake", m_intake.setIntakePowerConstant(IntakeConstants.kIntakeIntake));
    NamedCommands.registerCommand("StopIntake", m_intake.setIntakePowerConstant(0));
    NamedCommands.registerCommand("StartOuttake", m_intake.setIntakePowerConstant(IntakeConstants.kIntakeOuttake));

    NamedCommands.registerCommand("StartFeeder", m_shooter.setFeederPowerConstant(ShooterConstants.kFeederOn));
    NamedCommands.registerCommand("StartShooter", m_shooter.setShooterPowerConstant(ShooterConstants.kShooterOn));

    NamedCommands.registerCommand("StopFeeder", m_shooter.setFeederPowerConstant(0));
    NamedCommands.registerCommand("StopShooter", m_shooter.setShooterPowerConstant(0));

    NamedCommands.registerCommand("AimShooter", m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointSpeakerDefault));
    NamedCommands.registerCommand("StowShooter", m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));
    NamedCommands.registerCommand("FlatShooter", m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointFlat));

    // NamedCommands.registerCommand("AutoAim", new RunCommand(() -> {
    //       int tagId = (DriverStation.getAlliance().get() == Alliance.Blue) ? 7 : 4;
    //       m_shooter.setShooterAngleManual(-m_drivetrain.getAngleToSpeakerApriltag(tagId, 0.4318, m_shooter));
    //   }));

    NamedCommands.registerCommand("Handoff", 
      m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn)
      .andThen(new WaitCommand(0.5))
      .andThen(m_intake.setIntakePowerConstant(0.5))
      .andThen(m_shooter.setFeederPowerConstant(0.5))
      .andThen(new WaitCommand(0.25))
      .andThen(m_intake.setIntakePowerConstant(0))
      .andThen(m_shooter.setFeederPowerConstant(0))
    );

    NamedCommands.registerCommand("Shoot",
      m_shooter.setFeederPowerConstant(1)
      .andThen(new WaitCommand(1))
      .andThen(m_shooter.setFeederPowerConstant(0))
    );

    NamedCommands.registerCommand("Intake", 
      m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut)
      .andThen(m_intake.setIntakePowerConstant(IntakeConstants.kIntakeIntake))
    );

    m_autoChooser = AutoBuilder.buildAutoChooser("Preload Note Shooter");
    SmartDashboard.putData(m_autoChooser);

    m_driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(1);

    m_dualDriving = new DualDriver();
    m_singleDriving = new SingleDriver();

    configureBindings();
  }

  private void configureBindings() {
    // Driver Bindings
    m_drivetrain.setDefaultCommand(m_drivetrain.driveCommand(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
      () -> MathUtil.applyDeadband(-m_driverController.getRightY(), OIConstants.kDriveDeadband),
      () -> false //m_driverController.leftTrigger().getAsBoolean()
    ));

    m_singleDriving.setup();
  }



  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
