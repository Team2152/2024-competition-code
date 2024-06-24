package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class HandoffSequence extends SequentialCommandGroup {
    public HandoffSequence(Intake m_intake, Shooter m_shooter) {
        addCommands(
            m_shooter.setPivotCmd(ShooterConstants.Setpoints.kHandoff),
            m_intake.setPivotCmd(IntakeConstants.Setpoints.kStow),
            new WaitCommand(0.5),
            m_intake.setRollerCmd(0.5),
            m_shooter.setFeederCmd(0.5),
            new WaitCommand(0.25),
            m_intake.setRollerCmd(0),
            m_shooter.setFeederCmd(0)
        );
    }
}