package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase { 
    public final IntakePivot m_intakePivot;
    private final IntakeRollers m_intakeRollers;
    private final DigitalInput m_noteLimitSwitch;

    public Intake() {
        m_intakePivot = new IntakePivot(IntakeConstants.kPivotCanId, 15);
        m_intakeRollers = new IntakeRollers(IntakeConstants.kIntakeCanId); 
        m_noteLimitSwitch = new DigitalInput(IntakeConstants.kIntakeNoteLsPort);     
    }

    @Override
    public void periodic() {
        if (getNoteDetected() && SmartDashboard.getBoolean("Note Detection", true)) {
            setIntakePower(0);
            setIntakeAngle(IntakeConstants.kIntakeSetpointIn);
        }
    }

    public double getIntakeAngleDouble() {
        return m_intakePivot.getIntakeAngle().getValueAsDouble();
    }

    public StatusSignal<Double> getIntakeAngleRaw() {
        return m_intakePivot.getIntakeAngle();
    }

    public double getIntakePower() {
        return m_intakeRollers.getIntakePower();
    }

    public Boolean getNoteDetected() {
        return m_noteLimitSwitch.get();
    }

    public Command setIntakeAngle(double angle) {
        return m_intakePivot.setIntakeAngle(angle);
    }

    public Command setIntakePower(double power) {
        return m_intakeRollers.setIntakePower(power);
    }

    public Command setIntakePowerWithChecks(double power) {
        return m_intakeRollers.setIntakePowerWithChecks(power, getNoteDetected());
    }

    public Command setIntakePowerConstant(double power) {
        return m_intakeRollers.setIntakePowerConstant(power);
    }
}
