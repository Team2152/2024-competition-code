package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase {
  private final TalonFX m_pivotMotor;
  private final PIDController m_pivotPIDController;

  public IntakePivot(int pivotCanId) {
    m_pivotMotor = new TalonFX(pivotCanId);
    m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    m_pivotPIDController = new PIDController(
      IntakeConstants.kPivotP,
      IntakeConstants.kPivotI,
      IntakeConstants.kPivotD);
  }

  @Override
  public void periodic() {}

  public StatusSignal<Double> getIntakeAngle() {
    return m_pivotMotor.getPosition();
  }

  public Command setIntakeAngle(double targetAngle) {
    return run(() -> {
      double intakePivotOutput = m_pivotPIDController.calculate(getIntakeAngle().getValueAsDouble(), targetAngle);
      m_pivotMotor.set(intakePivotOutput);
    }); 
  }
}