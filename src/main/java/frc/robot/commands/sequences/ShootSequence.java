package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Shooter m_shooter) {
        addCommands(
            m_shooter.setShooterCmd(1, ShooterConstants.kShooterBias),
            new WaitCommand(1.25),
            m_shooter.setFeederCmd(1),
            new WaitCommand(1),
            m_shooter.setFeederCmd(0),
            m_shooter.setShooterCmd(.25, 0)
        );
    }
}