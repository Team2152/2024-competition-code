package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class RobotContainer {
  // public final Limelight m_frontCamera;
  // public final Limelight m_rearCamera;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;

  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  public final CommandXboxController m_operatorController;

  public final LEDs m_leds;


  public RobotContainer() {
    // m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);
    // m_rearCamera = new Limelight();

    m_drivetrain = new Drivetrain();
    m_intake = new Intake();
    m_shooter = new Shooter();

    m_leds = new LEDs(OIConstants.kLedPort, OIConstants.kLedLength);

    NamedCommands.registerCommand("DeployIntake", m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut));
    NamedCommands.registerCommand("RetractIntake", m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn));

    NamedCommands.registerCommand("StartIntake", m_intake.setIntakeAngle(IntakeConstants.kIntakeIntake));
    NamedCommands.registerCommand("StopIntake", m_intake.setIntakeAngle(0));
    NamedCommands.registerCommand("StartOuttake", m_intake.setIntakeAngle(IntakeConstants.kIntakeOuttake));

    NamedCommands.registerCommand("StartFeeder", m_shooter.setFeederPowerConstant(ShooterConstants.kFeederOn));
    NamedCommands.registerCommand("StartShooter", m_shooter.setShooterPowerConstant(ShooterConstants.kShooterOn));

    NamedCommands.registerCommand("StopFeeder", m_shooter.setFeederPowerConstant(0));
    NamedCommands.registerCommand("StopShooter", m_shooter.setShooterPowerConstant(0));

    NamedCommands.registerCommand("AimShooter", m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointSpeakerDefault));
    NamedCommands.registerCommand("StowShooter", m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));

    m_autoChooser = AutoBuilder.buildAutoChooser("Preload Note Shooter");
    SmartDashboard.putData(m_autoChooser);

    m_driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(1);

    configureBindings();
  }

  private void configureBindings() {
    // Driver Bindings
    m_drivetrain.setDefaultCommand(
      m_drivetrain.teleopDrive(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true,
        () -> m_driverController.rightBumper().getAsBoolean()
    ));

    m_driverController.leftBumper()
      .onTrue(m_drivetrain.setX());

    m_driverController.povUp()
      .whileTrue(m_drivetrain.zeroHeading());

    m_operatorController.a()
      .onTrue(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut))
      .onFalse(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn));

    m_operatorController.leftBumper()
      .onTrue(m_intake.setIntakePower(IntakeConstants.kIntakeIntake))
      .onFalse(m_intake.setIntakePower(0));

    m_operatorController.rightBumper()
    .onTrue(m_intake.setIntakePower(IntakeConstants.kIntakeOuttake))
    .onFalse(m_intake.setIntakePower(0));

    m_operatorController.povDown()
      .onTrue(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointSpeakerDefault));

    m_operatorController.povUp()
      .onTrue(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));

    m_operatorController.leftTrigger()
      .onTrue(m_shooter.setFeederPower(ShooterConstants.kFeederOn / 2))
      .onFalse(m_shooter.setFeederPower(0));

    m_operatorController.rightTrigger()
    .onTrue(m_shooter.setShooterPower(ShooterConstants.kShooterOn))
    .onFalse(m_shooter.setShooterPower(0));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
