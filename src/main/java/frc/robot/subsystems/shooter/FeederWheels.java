package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FeederWheels extends SubsystemBase{
    private final CANSparkMax m_leftFeederMotor;
    private final CANSparkMax m_rightFeederMotor;

    public FeederWheels(int leftMotorCanId, int rightMotorCanId) {
        m_leftFeederMotor = new CANSparkMax(leftMotorCanId, MotorType.kBrushless);
        m_rightFeederMotor = new CANSparkMax(rightMotorCanId, MotorType.kBrushless);

        m_leftFeederMotor.setInverted(ShooterConstants.kFeederInverted);
        m_rightFeederMotor.follow(m_leftFeederMotor, true);
    } 

    @Override
    public void periodic() {}

    public double getFeederPower() {
        return m_leftFeederMotor.get();
    }

    public Command setFeederPower(double power) {
        return run(() -> {
            m_leftFeederMotor.set(power);
        });
    }
}
