package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.PivotPID;

public class IntakePivot extends SubsystemBase{
    private final TalonFX m_pivotMotor;
    private final TalonFXConfiguration m_pivotMotorConfigs;
    private final MotionMagicVoltage m_request;

    public IntakePivot(int pivotMotorCanId, double gearRatio) {
        m_pivotMotor = new TalonFX(pivotMotorCanId);

        m_pivotMotor.setInverted(true);

        m_pivotMotorConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = m_pivotMotorConfigs.Slot0;
        slot0Configs.kS = PivotPID.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kG = PivotPID.kG; // gravityyyyyyyyyyy
        slot0Configs.kV = PivotPID.kV; // A velocity target of 1 rps results in 12.0 V output
        slot0Configs.kA = PivotPID.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = PivotPID.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = PivotPID.kI; // no output for integrated error
        slot0Configs.kD = PivotPID.kD; // A velocity error of 1 rps results in 0.1 V output
        m_pivotMotorConfigs.Feedback.SensorToMechanismRatio = gearRatio;
        m_pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        MotionMagicConfigs motionMagicConfigs = m_pivotMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotPID.kCruiseVelocity / gearRatio; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = PivotPID.kAcceleration / gearRatio; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = PivotPID.kJerk / gearRatio; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_pivotMotor.getConfigurator().apply(m_pivotMotorConfigs);
        m_request = new MotionMagicVoltage(0);
    } 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeCurrentPos", m_pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeCurrentSetpoint", m_request.Position);
    }

    public StatusSignal<Double> get() {
        return m_pivotMotor.getPosition();
    }

    public void set(double angle) {
        double intakeTargetPosition = Units.degreesToRotations(angle);
        m_pivotMotor.setControl(m_request.withPosition(intakeTargetPosition));
    }

    public void reset(double angle) {
        m_pivotMotor.setPosition(angle);
    }
}