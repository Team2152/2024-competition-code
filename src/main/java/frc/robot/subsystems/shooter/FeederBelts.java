package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FeederBelts extends SubsystemBase{
    private final CANSparkMax m_leftFeederMotor;
    private final CANSparkMax m_rightFeederMotor;

    public FeederBelts(int leftMotorCanId, int rightMotorCanId) {
        m_leftFeederMotor = new CANSparkMax(leftMotorCanId, MotorType.kBrushless);
        m_rightFeederMotor = new CANSparkMax(rightMotorCanId, MotorType.kBrushless);

        m_leftFeederMotor.setInverted(ShooterConstants.kFeederInverted);
        m_rightFeederMotor.follow(m_leftFeederMotor, true);
    } 

    @Override
    public void periodic() {}

    public double get() {
        return m_leftFeederMotor.get();
    }

    public void set(double speed) {
        m_leftFeederMotor.set(speed);
    }
}
