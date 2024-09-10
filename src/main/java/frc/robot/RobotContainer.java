package frc.robot;

import org.photonvision.common.hardware.VisionLEDMode;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NoteTracking;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class RobotContainer {
  // public final Limelight m_frontCamera;
  public final Limelight m_rearCamera;
  public final NoteTracking m_noteTracking;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;



  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  public final CommandXboxController m_operatorController;

  public final LEDs m_leds;

  // public final Orchestra m_music;

  
  public RobotContainer() {
    m_rearCamera = new Limelight(Constants.Vision.kRearCameraName, Constants.Vision.kRearRobotToCam);
    m_noteTracking = new NoteTracking();
    // m_frontCamera = new Limelight();

    m_drivetrain = new Drivetrain(m_rearCamera, m_noteTracking);
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

    NamedCommands.registerCommand("AutoAim", new RunCommand(() -> {
          int tagId = (DriverStation.getAlliance().get() == Alliance.Blue) ? 7 : 4;
          m_shooter.setShooterAngleManual(-m_drivetrain.getAngleToSpeakerApriltag(tagId, 0.4318, m_shooter));
      }));

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

    // m_music = new Orchestra();
    // m_music.loadMusic("music/tetris.chrp");
    // m_music.addInstrument(m_shooter.m_shooterPivot.m_pivotMotor);
    // m_music.addInstrument(m_intake.m_intakePivot.m_pivotMotor);
    // m_music.addInstrument(m_shooter.m_shooterWheels.m_leftShooterMotor);
    // m_music.addInstrument(m_shooter.m_shooterWheels.m_rightShooterMotor);
    // m_music.play();

    configureBindings();
  }

  private void configureBindings() {
    // Driver Bindings
    m_drivetrain.setDefaultCommand(
      m_drivetrain.teleopDrive(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.leftTrigger().getAsBoolean()
    ));

    // PathPlannerPath ampPath = PathPlannerPath.fromPathFile("Amp Aim");
    // m_driverController.a()
    //   .onTrue(
    //     new RunCommand(() -> {
    //       m_music.stop();
    //       m_music.loadMusic("music/tetris.chrp");
    //       m_music.play();
    //     })
    //   );

    m_driverController.start()
      .whileTrue(
        m_shooter.resetPivotMotor(0)
      );

    m_driverController.rightTrigger()
      .whileTrue(m_drivetrain.setX())
      .onTrue(
        m_leds.setColor(Color.kRed)
        .andThen(m_leds.setBlink(true)))
      .onFalse(
        m_leds.setColor(OIConstants.kLedOrange)
        .andThen(m_leds.setBlink(false)));

    // m_driverController.rightTrigger()
    //   .whileTrue(m_shooter.setShooter)

    m_driverController.leftBumper()
      .whileTrue(m_drivetrain.zeroHeading());

    m_driverController.povUp()
      .whileTrue(
        m_shooter.setPivotPower(-0.05))
      .onFalse(
        m_shooter.setPivotPower(0));

    m_driverController.povDown()
      .whileTrue(
        m_shooter.setPivotPower(0.05))
      .onFalse(
        m_shooter.setPivotPower(0));

      m_driverController.x()
        .toggleOnTrue(new RunCommand(() -> {
          m_leds.setBlink(true);
          m_leds.setColor(Color.kGreen);
          int tagId = (DriverStation.getAlliance().get() == Alliance.Blue) ? 7 : 4;
          m_shooter.setShooterAngleManual(-m_drivetrain.getAngleToSpeakerApriltag(tagId, 0.4318, m_shooter));
          m_drivetrain.faceHeadingManual(m_drivetrain.getHeadingFromApriltag(tagId, m_drivetrain.getPose()));
          m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.25);
          m_operatorController.getHID().setRumble(RumbleType.kRightRumble, 0.25);
          SmartDashboard.putBoolean("AUTO AIM", true);
      }))
      .toggleOnFalse(
        m_leds.setBlink(false)
        .andThen(m_leds.setColor(OIConstants.kLedOrange))
        .andThen(() -> {m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
          m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
        SmartDashboard.putBoolean("AUTO AIM", false);}
        
        ));

        m_driverController.y()
          .onTrue(m_noteTracking.toggleCmd());

        m_driverController.b()
          .toggleOnTrue(new RunCommand(() -> {
            int ampRotation = DriverStation.getAlliance().get() == Alliance.Blue ? 90 : -90;
            m_drivetrain.faceHeadingManual(ampRotation);
          }));
        
      // .onTrue(
      //   m_rearCamera.setLED(VisionLEDMode.kBlink)
      //   .andThen(new WaitCommand(2))
      //   .andThen(m_rearCamera.setLED(VisionLEDMode.kOff)));

    m_operatorController.a()
      .onTrue(
        m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut)
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
      m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointFlat));

    m_operatorController.povDown()
      .onTrue(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));

    m_operatorController.leftTrigger()
      .onTrue(m_shooter.setFeederPower(ShooterConstants.kFeederOn / 2))
      .onFalse(m_shooter.setFeederPower(0));

    m_operatorController.rightTrigger()
    .onTrue(
      m_leds.setColor(Color.kGreen)
      .andThen(m_leds.setBlink(true))
      .andThen(m_shooter.setShooterPower(ShooterConstants.kShooterOn))
    )
    .onFalse(
      m_leds.setColor(OIConstants.kLedOrange)
      .andThen(m_leds.setBlink(false))
      .andThen(m_shooter.setShooterPower(0)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
