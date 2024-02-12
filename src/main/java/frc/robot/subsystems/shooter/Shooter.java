package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterPivot m_shooterPivot;
    private final FeederWheels m_feederWheels;
    private final ShooterWheels m_shooterWheels;

    public Shooter() {
        m_shooterPivot = new ShooterPivot(ShooterConstants.kPivotCanId);
        m_feederWheels = new FeederWheels(ShooterConstants.kLeftFeederCanId, ShooterConstants.kRightFeederCanId);
        m_shooterWheels = new ShooterWheels(ShooterConstants.kLeftShooterCanId, ShooterConstants.kRightShooterCanId);
    }

    @Override
    public void periodic() {}

    public void setShooterAngle(double angle) {
        m_shooterPivot.setShooterAngle(angle);
    }

    public void setFeederPower(double power) {
        m_feederWheels.setFeederPower(power);
    }

    public void setShooterPower(double power) {
        m_shooterWheels.setShooterPower(power);
    }


    public double getShooterAngleDouble() {
        return m_shooterPivot.getShooterAngle().getValueAsDouble();
    }

    public StatusSignal<Double> getShooterAngleRaw() {
        return m_shooterPivot.getShooterAngle();
    }

    public double getFeederPower() {
        return m_feederWheels.getFeederPower();
    }

    public double getShooterPower() {
        return m_shooterWheels.getShooterPower();
    }
}