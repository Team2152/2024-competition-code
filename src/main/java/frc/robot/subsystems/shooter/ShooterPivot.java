package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase{
    private final TalonFX m_pivotMotor;
    private final PIDController m_pivotPIDController;

    public ShooterPivot(int pivotMotorCanId) {
        m_pivotMotor = new TalonFX(pivotMotorCanId);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotPIDController = new PIDController(
            ShooterConstants.kPivotP,
            ShooterConstants.kPivotI,
            ShooterConstants.kPivotD);
    } 

    @Override
    public void periodic() {}

    public StatusSignal<Double> getShooterAngle() {
        return m_pivotMotor.getPosition();
    }

    public Command setShooterAngle(double targetAngle) {
        return run(() -> {
            double intakePivotOutput = m_pivotPIDController.calculate(getShooterAngle().getValueAsDouble(), targetAngle);
            m_pivotMotor.set(intakePivotOutput);
        }); 
    }
}
