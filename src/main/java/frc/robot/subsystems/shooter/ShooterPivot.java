package frc.robot.subsystems.shooter;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;

public class ShooterPivot extends SubsystemBase{
    private final TalonFX m_pivotMotor;
    private final PIDController m_pivotPIDController;

    public ShooterPivot(int pivotMotorCanId) {
        m_pivotMotor = new TalonFX(pivotMotorCanId);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotPIDController = new PIDController(
            ShooterConstants.kPivotP,
            ShooterConstants.kPivotI,
            ShooterConstants.kPivotD);
    } 

    @Override
    public void periodic() {}

    public StatusSignal<Double> getShooterAngle() {
        return m_pivotMotor.getPosition();
    }

    public Command setShooterAngle(double targetAngle) {
        return run(() -> {
            double intakePivotOutput = m_pivotPIDController.calculate(getShooterAngle().getValueAsDouble(), targetAngle);
            m_pivotMotor.set(intakePivotOutput);
        }); 
    }

    public double getNeededShooterAngleFromApriltag(Limelight vision, int apriltagId, Pose2d currentPose, double heightAboveTag) {
        Optional<EstimatedRobotPose> estPose = vision.getEstimatedPose();
        if (estPose.isPresent()) {
            List<PhotonTrackedTarget> targets = vision.result.getTargets();
            for (PhotonTrackedTarget tgt : targets) {
                if (tgt.getFiducialId() == apriltagId) {
                    var tagPose = vision.m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isPresent()) {
                        Pose2d tagPose2d = tagPose.get().toPose2d();
                        double distanceToTarget = tagPose2d.getTranslation().getDistance(currentPose.getTranslation());
                        double angleToAimRadians = Math.atan2(heightAboveTag, distanceToTarget);

                        double angleToAimDegrees = Math.toDegrees(angleToAimRadians);
                        setShooterAngle(angleToAimDegrees);
                            
                        return angleToAimDegrees;
                    }
                }
            }
        }
        return angleToAimDegrees;
    }

    private double calculateAngleForTarget(double targetHeight, double distance) {
        final double g = 32.174; // acceleration due to gravity in ft/s^2
        final double initialHeight = 0.0; // initial height of the projectile (assuming it's launched horizontally)
        final double initialVelocity = Math.sqrt((distance * g) / (2 * (targetHeight - initialHeight)));
        return Math.atan((initialVelocity * initialVelocity + Math.sqrt(initialVelocity * initialVelocity * initialVelocity * initialVelocity - g * ((g * distance * distance) + (2 * (targetHeight - initialHeight) * initialVelocity * initialVelocity)))) / (g * distance));
    }
}