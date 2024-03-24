package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterPivot m_shooterPivot;
    private final FeederWheels m_feederWheels;
    private final ShooterWheels m_shooterWheels;

    public Shooter() {
        m_shooterPivot = new ShooterPivot(ShooterConstants.kPivotCanId,250);
        m_feederWheels = new FeederWheels(ShooterConstants.kLeftFeederCanId, ShooterConstants.kRightFeederCanId);
        m_shooterWheels = new ShooterWheels(ShooterConstants.kLeftShooterCanId, ShooterConstants.kRightShooterCanId);
    }

    @Override
    public void periodic() {}

    public Command setPivotPower(double power) {
        return m_shooterPivot.setPivotPower(power);
    }

    public Command setShooterAngle(double angle) {
        return m_shooterPivot.setShooterAngle(angle);
    }

    public void setShooterAngleManual(double angle) {
        m_shooterPivot.setShooterAngleManual(angle);
    }

    public Command setFeederPower(double power) {
        return m_feederWheels.setFeederPower(power);
    }

    public Command setFeederPowerConstant(double power) {
        return m_feederWheels.setFeederPowerConstant(power);
    }

    public Command setShooterPowerConstant(double power) {
        return m_shooterWheels.setShooterPowerConstant(power);
    }

    public Command setShooterPower(double power) {
        return m_shooterWheels.setShooterPower(power);
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

    public Command setShooterAngleSupplier(DoubleSupplier angle) {
        return m_shooterPivot.setShooterAngle(angle.getAsDouble());
    }

    public Command resetPivotMotor(double angle) {
        return m_shooterPivot.resetPivotMotor(angle);
    }
}
