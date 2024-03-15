package frc.robot;

import org.photonvision.common.hardware.VisionLEDMode;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class RobotContainer {
  // public final Limelight m_frontCamera;
  public final Limelight m_rearCamera;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;

  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  public final CommandXboxController m_operatorController;

  public final LEDs m_leds;


  public RobotContainer() {
    m_rearCamera = new Limelight(Constants.Vision.kRearCameraName, Constants.Vision.kRearRobotToCam);
    // m_frontCamera = new Limelight();

    m_drivetrain = new Drivetrain(m_rearCamera);
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

    m_driverController.rightTrigger()
      .whileTrue(m_drivetrain.setX())
      .onTrue(
        m_leds.setColor(Color.kFirstRed)
        .andThen(m_leds.setBlink(true)))
      .onFalse(
        m_leds.setColor(OIConstants.kLedOrange)
        .andThen(m_leds.setBlink(false)));

    m_driverController.povUp()
      .whileTrue(m_drivetrain.zeroHeading());

    m_driverController.a()
      .onTrue(
        m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointHang)
        .andThen(m_leds.setColor(Color.kGreen))
        .andThen(m_leds.setBlink(true))
      ).onFalse(
        m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow)
        .andThen(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointHang))
        .andThen(m_leds.setBlink(false))
        .andThen(new WaitCommand(3))
        .andThen(m_leds.setColor(OIConstants.kLedOrange))
        .andThen(m_leds.setBlink(false)));

    m_operatorController.a()
      .onTrue(
        m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut)
        .andThen(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow))
        .andThen(m_leds.setColor(Color.kRed))
      )
      .onFalse(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn)
      .andThen(m_leds.setColor(Color.kBlue))
      .andThen(new WaitCommand(3))
      .andThen(m_leds.setColor(OIConstants.kLedOrange)));

    m_operatorController.x().onTrue(
      m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointAmp)
      .andThen(m_leds.setColor(Color.kRed))
      .andThen(m_leds.setBlink(true))
    ).onFalse(
      m_intake.setIntakePowerConstant(IntakeConstants.kIntakeOuttake)
      .andThen(m_leds.setBlink(false))
      .andThen(new WaitCommand(0.3))
      .andThen(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn))
      .andThen(m_intake.setIntakePowerConstant(0))
      .andThen(m_leds.setColor(OIConstants.kLedOrange))
    );

      // .onTrue(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut / 3)
      // .andThen(m_leds.setColor(Color.kYellow))
      // .andThen(new WaitCommand(2.3))
      // .andThen(m_intake.setIntakePowerConstant(1))
      // .andThen(new WaitCommand(1))
      // .andThen(m_intake.setIntakePowerConstant(0))  
      // .andThen(new WaitCommand(0.1))
      // .andThen(m_intake.setIntakeAngle(0))   
      // .andThen(m_leds.setColor(OIConstants.kLedOrange))

    m_operatorController.rightBumper()
      .onTrue(
        m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn)
        .andThen(new WaitCommand(0.5))
        .andThen(m_intake.setIntakePowerConstant(0.5))
        .andThen(m_shooter.setFeederPowerConstant(0.5))
        .andThen(new WaitCommand(0.2))
        .andThen(m_intake.setIntakePowerConstant(0))
        .andThen(m_shooter.setFeederPowerConstant(0))
        .andThen(m_leds.setColor(Color.kGreen))
      );

    m_operatorController.leftBumper()
      .whileTrue(m_intake.setIntakePower(-1));

    m_operatorController.b()
    .whileTrue(m_intake.setIntakePower(IntakeConstants.kIntakeOuttake));

    m_operatorController.povLeft().onTrue(
      m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointSpeakerDefault)
      .andThen(m_leds.setBlink(false)));
  
    m_operatorController.povUp().onTrue(
      m_shooter.setShooterAngle(-90)
      .andThen(m_leds.setColor(OIConstants.kLedOrange)));

    m_operatorController.povDown()
      .onTrue(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));

    m_operatorController.leftTrigger()
      .onTrue(m_shooter.setFeederPower(ShooterConstants.kFeederOn / 2))
      .onFalse(m_shooter.setFeederPower(0));

    m_operatorController.rightTrigger()
    .onTrue(m_shooter.setShooterPower(ShooterConstants.kShooterOn)
    .andThen(m_leds.setColor(Color.kPurple)))
    .onFalse(m_shooter.setShooterPower(0)
    .andThen(m_leds.setColor(OIConstants.kLedOrange)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
