package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWheels extends SubsystemBase{
    private final TalonFX m_leftShooterMotor;
    private final TalonFX m_rightShooterMotor;

    public ShooterWheels(int leftMotorCanId, int rightMotorCanId) {
        m_leftShooterMotor = new TalonFX(leftMotorCanId);
        m_rightShooterMotor = new TalonFX(rightMotorCanId);

        m_rightShooterMotor.setInverted(true);
    } 

    @Override
    public void periodic() {}
    public double getShooterPower() {
        return m_leftShooterMotor.get();
    }

    public void set(double power, double bias) {
        m_leftShooterMotor.set(power);
        m_rightShooterMotor.set(Math.max(0, power - bias));
    }
}
