package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollers extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;

  public IntakeRollers(int intakeCanId) {
    m_intakeMotor = new CANSparkMax(intakeCanId, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeAmps);
    m_intakeMotor.setInverted(IntakeConstants.kIntakeInverted);
    m_intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {}

  public Command setIntakePower(double power) {
    return runEnd(
      () -> m_intakeMotor.set(power),
      () -> m_intakeMotor.set(0)
    ); 
  }

  public Command setIntakePowerConstant(double power) {
    return runOnce(
      () -> m_intakeMotor.set(power)
    ); 
  }

  public Command setIntakePowerWithChecks(double power, boolean note) {
    return runEnd(
      () -> {
        if (note) {m_intakeMotor.set(power);}
      },
      () -> m_intakeMotor.set(0)
    ); 
  }

  public double getIntakePower() {
    return m_intakeMotor.get();
  }
}

