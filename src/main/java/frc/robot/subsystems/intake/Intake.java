package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Intake extends SubsystemBase {
    private final TalonFX m_pivotMotor;
    private final PIDController m_pivotPIDController;
    private final CANSparkMax m_intakeMotor;

    
    public Intake() {
        m_pivotMotor = new TalonFX(Constants.Intake.kPivotCanId);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotPIDController = new PIDController(
            Constants.Intake.kPivotP,
            Constants.Intake.kPivotI,
            Constants.Intake.kPivotD);

        m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeCanId, MotorType.kBrushless);
        m_intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);
        m_intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeAmps);
        m_intakeMotor.setInverted(Constants.Intake.kIntakeInverted);
        m_intakeMotor.burnFlash();
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot", getIntakeAngle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Speed", getIntakePower());
    }

    public double getIntakePower() {
        return m_intakeMotor.get();
    }

    public StatusSignal<Double> getIntakeAngle() {
        return m_pivotMotor.getPosition();
    }

    public Command setIntakePower(double power) {
        return run(() -> {
            m_intakeMotor.set(power);
        }); 
    }

    public Command setIntakeAngle(double targetAngle) {
        return run(() -> {
            double intakePivotOutput = m_pivotPIDController.calculate(getIntakeAngle().getValueAsDouble(), targetAngle);
            m_pivotMotor.set(intakePivotOutput);
        }); 
    }
}
