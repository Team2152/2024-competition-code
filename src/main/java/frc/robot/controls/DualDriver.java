package frc.robot.controls;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class DualDriver extends RobotContainer {
    public DualDriver() {

    }
    
    public void setup() {
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

        // m_driverController.x()
        //   .toggleOnTrue(new RunCommand(() -> {
        //     m_leds.setBlink(true);
        //     m_leds.setColor(Color.kGreen);
        //     int tagId = (DriverStation.getAlliance().get() == Alliance.Blue) ? 7 : 4;
        //     m_shooter.setShooterAngleManual(-m_drivetrain.getAngleToSpeakerApriltag(tagId, 0.4318, m_shooter));
        //     m_drivetrain.faceHeadingManual(m_drivetrain.getHeadingFromApriltag(tagId, m_drivetrain.getPose()));
        //     m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.25);
        //     m_operatorController.getHID().setRumble(RumbleType.kRightRumble, 0.25);
        //     SmartDashboard.putBoolean("AUTO AIM", true);
        // }))
        // .toggleOnFalse(
        //   m_leds.setBlink(false)
        //   .andThen(m_leds.setColor(OIConstants.kLedOrange))
        //   .andThen(() -> {m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        //     m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
        //   SmartDashboard.putBoolean("AUTO AIM", false);}
            
        //   ));

            m_driverController.y()
            .onTrue(m_noteTracking.toggleCmd());

            // m_driverController.b()
            //   .toggleOnTrue(new RunCommand(() -> {
            //     int ampRotation = DriverStation.getAlliance().get() == Alliance.Blue ? 90 : -90;
            //     m_drivetrain.faceHeadingManual(ampRotation);
            //   }));
            
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
}
