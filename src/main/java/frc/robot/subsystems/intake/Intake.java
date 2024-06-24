package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private IntakeRollers m_rollers;
    private IntakePivot m_pivot;

    public Intake() {
        m_rollers = new IntakeRollers(IntakeConstants.kIntakeCanId);
        m_pivot = new IntakePivot(IntakeConstants.kPivotCanId, IntakeConstants.kPivotRatio);
    }

    public Command setRollerCmd(double speed) {
        return runOnce(() ->
            setRoller(speed)
        );
    }

    public Command setRollerCmdEnd(double speed) {
        return runEnd(
            () -> setRoller(speed),
            () -> setRoller(0)
        ); 
    }

    public void setRoller(double speed) {
        m_rollers.set(speed);
    }

    
    public Command setPivotCmd(double angle) {
        return runOnce(() ->
            setPivot(angle)
        );
    }

    public void setPivot(double angle) {
        m_pivot.set(angle);
    }
}
