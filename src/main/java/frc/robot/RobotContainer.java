package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
  public final Limelight m_frontCamera;
  // public final Limelight m_rearCamera;

  public final Drivetrain m_drivetrain;
  public final Intake m_intake;
  public final Shooter m_shooter;

  public final SendableChooser<Command> m_autoChooser;


  public final CommandXboxController m_driverController;
  // public final CommandXboxController m_operatorController;


  public RobotContainer() {
    m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);
    // m_rearCamera = new Limelight();

    m_drivetrain = new Drivetrain(m_frontCamera);
    m_intake = new Intake();
    m_shooter = new Shooter();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);

    m_driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);

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
      .whileTrue(
          m_drivetrain.setX()
      );

    m_driverController.start()
      .onTrue(
        m_drivetrain.zeroHeading()
    );
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
