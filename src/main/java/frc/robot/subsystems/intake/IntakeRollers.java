package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollers extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;

  public IntakeRollers(int intakeCanId) {
    m_intakeMotor = new CANSparkMax(intakeCanId, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
    m_intakeMotor.setInverted(IntakeConstants.kIntakeInverted);
    m_intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {}

  public void set(double power) {
    m_intakeMotor.set(power);
  }

  public double get() {
    return m_intakeMotor.get();
  }
}

