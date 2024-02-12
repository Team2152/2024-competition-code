// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
  public final Limelight m_frontCamera;
  // public final Limelight m_rearCamera;

  public final Drivetrain m_drivetrain;
  // public final Intake m_intake;
  // public final Shooter m_shooter;


  public final CommandXboxController m_driverController;
  // public final CommandXboxController m_operatorController;


  public RobotContainer() {
    m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);
    // m_rearCamera = new Limelight();

    m_drivetrain = new Drivetrain(m_frontCamera);
    // m_intake = new Intake();
    // m_shooter = new Shooter();


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
    return Commands.print("No autonomous command configured");
  }
}
