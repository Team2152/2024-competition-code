package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase{
    public final TalonFX m_pivotMotor;
    private final TalonFXConfiguration m_pivotMotorConfigs;
    private final MotionMagicVoltage m_request;

    public ShooterPivot(int pivotMotorCanId, double gearRatio) {
        m_pivotMotor = new TalonFX(pivotMotorCanId);

        m_pivotMotor.setInverted(true);

        m_pivotMotorConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = m_pivotMotorConfigs.Slot0;
        slot0Configs.kS = 0.25;// Add 0.25 V output to overcome static friction
        slot0Configs.kV = 2.0; // A velocity target of 1 rps results in 12.0 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 25.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.2; // A velocity error of 1 rps results in 0.1 V output
        m_pivotMotorConfigs.Feedback.SensorToMechanismRatio = gearRatio;
        m_pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        MotionMagicConfigs motionMagicConfigs = m_pivotMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 300 / gearRatio; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 120 / gearRatio; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 300 / gearRatio; // Target jerk of 1600 rps/s/s (0.1 seconds)

        /* How to tune pivot configs
         * 
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
         * 
         * kS = Increase voltage until movement.
         */

        m_pivotMotor.getConfigurator().apply(m_pivotMotorConfigs);
        m_request = new MotionMagicVoltage(0); // default pivot position
    } 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterCurrentPos", m_pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterCurrentSetpoint", m_request.Position);

        SmartDashboard.putNumber("ShooterCurrentVel", m_pivotMotor.getVelocity().getValueAsDouble());
    }

    public StatusSignal<Double> getShooterAngle() {
        return m_pivotMotor.getPosition();
    }

    public Command setShooterAngle(double angle) {
        return runOnce(() -> {
            setShooterAngleManual(angle);
        }); 
    }

    public void setShooterAngleManual(double angle) {
        double targetAngle = angle;
          
        if (targetAngle > ShooterConstants.kPositiveShooterLimit) {
            targetAngle = ShooterConstants.kPositiveShooterLimit;
        } else if (targetAngle < ShooterConstants.kNegativeShooterLimit) {
            targetAngle = ShooterConstants.kNegativeShooterLimit;
        }
            
        double intakeTargetPosition = Units.degreesToRotations(targetAngle);
        m_pivotMotor.setControl(m_request.withPosition(intakeTargetPosition));
    }

    public Command setPivotPower(double power) {
        return runOnce(() -> m_pivotMotor.set(power));
    }

    public Command resetPivotMotor(double angle) {
        return runOnce(() -> m_pivotMotor.setPosition(angle));
    }

    public void resetPivotMotorManual(double angle) {
        m_pivotMotor.setPosition(angle);
    }
}