package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWheels extends SubsystemBase{
  private final TalonFX m_leftShooterMotor;
  private final TalonFX m_rightShooterMotor;
  private final Follower m_shooterFollower;

  public ShooterWheels(int leftMotorCanId, int rightMotorCanId) {
    m_leftShooterMotor = new TalonFX(leftMotorCanId);
    m_rightShooterMotor = new TalonFX(rightMotorCanId);
    m_shooterFollower = new Follower(leftMotorCanId, true);
    m_rightShooterMotor.setControl(m_shooterFollower);
  } 

  @Override
  public void periodic() {}
  public double getShooterPower() {
    return m_leftShooterMotor.get();
  }

  public Command setShooterPower(double power) {
    return run(() -> {
      m_leftShooterMotor.set(power);
    });
  }
}
