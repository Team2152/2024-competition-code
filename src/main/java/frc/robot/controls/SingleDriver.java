package frc.robot.controls;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class SingleDriver extends RobotContainer {    
    public SingleDriver() {
        
    }
    
    public void setup() {
        m_driverController.leftBumper()
        .onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
        
        m_driverController.start()
        .whileTrue(
            m_shooter.resetPivotMotor(0)
        );

        m_driverController.leftBumper()
        .onTrue(Commands.runOnce(m_drivetrain::zeroGyro));

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

        m_driverController.y()
        .onTrue(m_noteTracking.toggleCmd());

        m_driverController.a()
        .onTrue(
            m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointOut)
        )
        .onFalse(
            m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn)
        );

        m_driverController.x().onTrue(
            m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointAmp)
        ).onFalse(
            m_intake.setIntakePowerConstant(IntakeConstants.kIntakeOuttake)
            .andThen(new WaitCommand(0.3))
            .andThen(m_intake.setIntakeAngle(IntakeConstants.kIntakeSetpointIn))
            .andThen(m_intake.setIntakePowerConstant(0))
        );

        m_driverController.rightBumper()
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

        m_driverController.b()
        .whileTrue(m_intake.setIntakePower(IntakeConstants.kIntakeOuttake));

        m_driverController.leftTrigger()
        .whileTrue(m_intake.setIntakePower(IntakeConstants.kIntakeIntake));

        m_driverController.povRight().onTrue(
        m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointSpeakerDefault)
        .andThen(m_leds.setBlink(false)));

        m_driverController.povRight()
        .onTrue(m_shooter.setShooterAngle(ShooterConstants.kShooterSetpointStow));

        m_driverController.rightTrigger()
        .onTrue(
            m_shooter.setShooterPower(ShooterConstants.kShooterOn)
            .andThen(new WaitCommand(1.5))
            .andThen(m_shooter.setFeederPower(ShooterConstants.kShooterOn))
            .andThen(new WaitCommand(2))
            .andThen(m_shooter.setFeederPower(0))
            .andThen(m_shooter.setShooterPower(0))
        );
    }
}
