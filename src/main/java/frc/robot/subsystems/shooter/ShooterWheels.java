package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWheels extends SubsystemBase{
  public final TalonFX m_leftShooterMotor;
  public final TalonFX m_rightShooterMotor;
  // private final Follower m_shooterFollower;

  public ShooterWheels(int leftMotorCanId, int rightMotorCanId) {
    m_leftShooterMotor = new TalonFX(leftMotorCanId);
    m_rightShooterMotor = new TalonFX(rightMotorCanId);
    // m_shooterFollower = new Follower(leftMotorCanId, true);
    // m_rightShooterMotor.setControl(m_shooterFollower);
    m_rightShooterMotor.setInverted(true);
  } 

  @Override
  public void periodic() {}
  public double getShooterPower() {
    return m_leftShooterMotor.get();
  }

  public Command setShooterPowerConstant(double power) {
    return runOnce(() -> {
      m_leftShooterMotor.set(power);
      if (power <= 0.2) {
        m_rightShooterMotor.set(0);
      } else {
        m_rightShooterMotor.set(power - 0.2);
      }
    });
  }

  public Command setShooterPower(double power) {
    return runEnd(
      () -> {m_leftShooterMotor.set(power);m_rightShooterMotor.set(power - 0.1);},
      () -> {m_leftShooterMotor.set(0);m_rightShooterMotor.set(0);}
    ); 
  }
}
