package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;

public class EjectSequence extends SequentialCommandGroup {
    public EjectSequence(Shooter m_shooter) {
        addCommands(
            m_shooter.setFeederCmd(1),
            m_shooter.setShooterCmd(1, 0),
            new WaitCommand(.5),
            m_shooter.setFeederCmd(0),
            m_shooter.setShooterCmd(.25, 0)
        );
    }
}