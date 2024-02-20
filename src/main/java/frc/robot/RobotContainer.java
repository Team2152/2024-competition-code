package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
//import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
  // public final Limelight m_frontCamera;
  // public final Limelight m_rearCamera;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;

  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  //public final CommandXboxController m_operatorController;

  // public final LEDs m_leds;
  // public final DigitalInput m_intakeLimitSwitch;
  

  public RobotContainer() {
    // m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);
    // m_rearCamera = new Limelight();

    m_drivetrain = new Drivetrain();
    m_intake = new Intake();
    m_shooter = new Shooter();

    // m_leds = new LEDs();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);

    m_driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
    //m_operatorController = new CommandXboxController(1);

    // m_intakeLimitSwitch = new DigitalInput(Constants.IntakeConstants.kIntakeNoteLsPort);

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

    // new Trigger(m_intakeLimitSwitch::get)
    //   .onTrue(m_intake.setIntakeAngle(0), m_leds.setColor()
    // );

    m_driverController.leftBumper()
      .whileTrue(
          m_drivetrain.setX()
      );

    m_driverController.start()
      .onTrue(
        m_drivetrain.zeroHeading()
    );

    m_driverController.y()
      .onTrue(
        m_intake.setIntakeAngle(0)
        //m_shooter.setShooterAngle(55)
      ).onFalse(
        m_intake.setIntakeAngle(90)
        //m_shooter.setShooterAngle(0)
      );

    m_driverController.a()
      .onTrue(m_intake.setIntakePower(-1))
      .onFalse(m_intake.setIntakePower(0));

    m_driverController.b()
      .onTrue(m_shooter.setShooterPower(1))
      .onFalse(m_shooter.setShooterPower(0));

    m_driverController.x()
      .onTrue(m_shooter.setFeederPower(-1))
      .onFalse(m_shooter.setFeederPower(0));

  //   m_operatorController.a()
  //     .onTrue(m_intake.setIntakePower(-1))
  //     .onFalse(m_intake.setIntakePower(0));

  //   m_operatorController.b()
  //     .onTrue(m_shooter.setShooterPower(1))
  //     .onFalse(m_shooter.setShooterPower(0));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
