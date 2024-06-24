// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OIConstants.LedColors;
import frc.robot.Constants.VisionConstants.RearCamera;
import frc.robot.commands.sequences.EjectSequence;
import frc.robot.commands.sequences.HandoffSequence;
import frc.robot.commands.sequences.ShootSequence;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Autoaim;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
  public final Limelight m_rearCamera;
  
  public final Drivetrain m_drivetrain;
  public final Autoaim m_autoaim;

  public final Shooter m_shooter;
  public final Intake m_intake;
  
  public final LEDs m_leds;

  public final SendableChooser<Command> m_autoChooser;

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;


  public RobotContainer() {
    m_rearCamera = new Limelight(RearCamera.kCameraName, RearCamera.kCameraOffset, RearCamera.kSingleTagStdDevs, RearCamera.kMultiTagStdDevs);
    m_drivetrain = new Drivetrain(m_rearCamera);

    m_shooter = new Shooter();
    m_intake = new Intake();

    m_leds = new LEDs(OIConstants.kLedPwmPort, OIConstants.kLedLength);
    m_autoaim = new Autoaim(m_drivetrain, m_shooter);

    m_autoChooser = AutoBuilder.buildAutoChooser("Preload Note Shooter");
    SmartDashboard.putData(m_autoChooser);

    m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


    NamedCommands.registerCommand("DeployIntake", m_intake.setPivotCmd(IntakeConstants.Setpoints.kIntake));
    NamedCommands.registerCommand("RetractIntake", m_intake.setPivotCmd(IntakeConstants.Setpoints.kStow));

    NamedCommands.registerCommand("StartIntake", m_intake.setRollerCmd(IntakeConstants.kIntakeIntake));
    NamedCommands.registerCommand("StopIntake", m_intake.setRollerCmd(0));
    NamedCommands.registerCommand("StartOuttake", m_intake.setRollerCmd(IntakeConstants.kIntakeOuttake));

    NamedCommands.registerCommand("StartFeeder", m_shooter.setFeederCmd(ShooterConstants.kFeederOn));
    NamedCommands.registerCommand("StartShooter", m_shooter.setFeederCmd(ShooterConstants.kShooterOn));

    NamedCommands.registerCommand("StopFeeder", m_shooter.setFeederCmd(0));
    NamedCommands.registerCommand("StopShooter", m_shooter.setFeederCmd(0));

    NamedCommands.registerCommand("AimShooter", m_shooter.setPivotCmd(ShooterConstants.Setpoints.kHandoff));
    NamedCommands.registerCommand("StowShooter", m_shooter.setPivotCmd(ShooterConstants.Setpoints.kStow));
    NamedCommands.registerCommand("FlatShooter", m_shooter.setPivotCmd(ShooterConstants.Setpoints.kFlat));

    NamedCommands.registerCommand("AutoAim", m_autoaim.autoaimCmd());

    NamedCommands.registerCommand("Handoff", new HandoffSequence(m_intake, m_shooter));

    NamedCommands.registerCommand("Shoot", new ShootSequence(m_shooter));


    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      m_drivetrain.teleopDrive(
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.rightTrigger().getAsBoolean()
      ));

    // Track Speaker
    m_driverController.a()
    .toggleOnTrue(
      m_autoaim.autoaimCmd()
      .andThen(m_leds.setColor(LedColors.kReady))
      .andThen(m_leds.setBlink(true))
    ).toggleOnFalse(
      m_leds.setColor(LedColors.kDefault)
      .andThen(m_leds.setBlink(false))
    );

    // Swerve Brake
    m_driverController.leftTrigger()
    .whileTrue(m_drivetrain.setX());

    // Zero Gyro
    m_driverController.leftBumper()
    .whileTrue(m_drivetrain.zeroHeadingCmd());

    // Shooter Manual Control Up
    m_driverController.povUp()
    .whileTrue(
      m_shooter.setPowerCmd(-0.05))
    .onFalse(
      m_shooter.setPowerCmd(0));

    // Shooter Manual Control Down
    m_driverController.povDown()
    .whileTrue(
      m_shooter.setPowerCmd(0.05))
    .onFalse(
      m_shooter.setPowerCmd(0));

    // Alert Other Driver
    m_driverController.x()
    .onTrue(new InstantCommand(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.75)))
    .onFalse(new InstantCommand(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));


    // Intake
    m_operatorController.a()
    .onTrue(
      m_intake.setPivotCmd(IntakeConstants.Setpoints.kIntake)
      .andThen(m_leds.setBlink(true))
    )
    .onFalse(m_intake.setPivotCmd(IntakeConstants.Setpoints.kStow)
      .andThen(m_leds.setBlink(false))
    );

    // Intake Rollers
    m_operatorController.leftBumper()
    .whileTrue(m_intake.setRollerCmdEnd(-1));

    // Outtake Rollers
    m_operatorController.rightBumper()
    .whileTrue(m_intake.setRollerCmdEnd(1));

    // Handoff
    m_operatorController.x()
    .onTrue(
      new HandoffSequence(m_intake, m_shooter)
      .andThen(m_leds.setColor(LedColors.kReady))
    );

    // Amp
    m_operatorController.y()
    .onTrue(
      m_intake.setPivotCmd(IntakeConstants.Setpoints.kAmp)
      .andThen(m_leds.setColor(LedColors.kWaiting))
      .andThen(m_leds.setBlink(true))
    ).onFalse(
      m_intake.setRollerCmd(IntakeConstants.kIntakeOuttake)
      .andThen(m_leds.setBlink(false))
      .andThen(new WaitCommand(0.3))
      .andThen(m_intake.setPivotCmd(IntakeConstants.Setpoints.kStow))
      .andThen(m_intake.setRollerCmd(0))
      .andThen(m_leds.setColor(LedColors.kDefault))
    );

    // Alert Other Driver
    m_operatorController.b()
    .onTrue(new InstantCommand(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.75)))
    .onFalse(new InstantCommand(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));

    // Shoot
    m_operatorController.rightTrigger()
    .onTrue(
      new ShootSequence(m_shooter)
      .andThen(m_leds.setColor(LedColors.kDefault))
    );

    // Eject
    m_operatorController.rightTrigger()
    .onTrue(
      new EjectSequence(m_shooter)
      .andThen(m_leds.setColor(LedColors.kDefault))
    );

    // Horizontal Angle
    m_operatorController.povUp()
    .onTrue(m_shooter.setPivotCmd(ShooterConstants.Setpoints.kFlat));

    // Handoff Angle
    m_operatorController.povUp()
    .onTrue(m_shooter.setPivotCmd(ShooterConstants.Setpoints.kHandoff));

    // Vertical Angle
    m_operatorController.povUp()
    .onTrue(m_shooter.setPivotCmd(ShooterConstants.Setpoints.kStow));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
