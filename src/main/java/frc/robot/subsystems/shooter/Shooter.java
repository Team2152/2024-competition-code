package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private FeederBelts m_feeder;
    private ShooterWheels m_flywheels;
    private ShooterPivot m_pivot;

    public Shooter() {
        m_feeder = new FeederBelts(ShooterConstants.kLeftFeederCanId, ShooterConstants.kRightFeederCanId);
        m_flywheels = new ShooterWheels(ShooterConstants.kLeftShooterCanId, ShooterConstants.kRightShooterCanId);
        m_pivot = new ShooterPivot(ShooterConstants.kPivotCanId, ShooterConstants.kPivotRatio);
    }

    public Command setFeederCmd(double speed) {
        return runOnce(() ->
            setFeeder(speed)
        );
    }

    public Command setFeederCmdEnd(double speed) {
        return runEnd(
            () -> setFeeder(speed),
            () -> setFeeder(0)
        ); 
    }

    public void setFeeder(double speed) {
        m_feeder.set(speed);
    }


    public Command setShooterCmd(double speed, double bias) {
        return runOnce(() ->
            setShooter(speed, bias)
        );
    }

    public Command setShooterCmdEnd(double speed, double bias) {
        return runEnd(
            () -> setShooter(speed, bias),
            () -> setShooter(0, 0)
        ); 
    }

    public void setShooter(double speed, double bias) {
        m_flywheels.set(speed, bias);
    }

    
    public Command setPivotCmd(double angle) {
        return runOnce(() ->
            setPivot(angle)
        );
    }

    public void setPivot(double angle) {
        m_pivot.set(angle);
    }


    public void setPower(double power) {
        m_pivot.setPower(power);
    }

    public Command setPowerCmd(double power) {
        return runOnce(() -> 
            m_pivot.setPower(power)
        );
    }
}
