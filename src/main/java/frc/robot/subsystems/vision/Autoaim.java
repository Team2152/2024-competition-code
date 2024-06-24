package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.utils.Helpers;

public class Autoaim extends SubsystemBase {
    private Drivetrain m_drivetrain;
    private Shooter m_shooter;

    public Autoaim(Drivetrain m_drivetrain, Shooter m_shooter) {

    }

    private double calculateDrivebaseAngle(int apriltag) {
        Pose3d requestedTagPose = VisionConstants.kTagLayout.getTagPose(apriltag).get();
        Pose2d currentPose = m_drivetrain.getPose();
        
        double headingToTarget = Math.atan2(
        requestedTagPose.getY() - currentPose.getY(),
        requestedTagPose.getX() - currentPose.getX()
        );

        return Units.radiansToDegrees(headingToTarget);
    }

    private double calculateShooterAngle(int apriltagId) {
        Pose3d requestedTagPose = VisionConstants.kTagLayout.getTagPose(apriltagId).get();
        Pose2d currentPose = m_drivetrain.getPose();

        double distanceToTarget = Math.sqrt(
            Math.pow(currentPose.getX() - requestedTagPose.getX(), 2) +
            Math.pow(currentPose.getY() - requestedTagPose.getY(), 2) +
            Math.pow(ShooterConstants.kShooterHeight - requestedTagPose.getZ(), 2)
        );

        distanceToTarget = Math.max(AutonomousConstants.kClosestDistance, 
            Math.min(distanceToTarget, AutonomousConstants.kFarthestDistance));

        double factor = (distanceToTarget - AutonomousConstants.kClosestDistance) / (AutonomousConstants.kFarthestDistance - AutonomousConstants.kClosestDistance);
        return AutonomousConstants.kClosestAngle + factor * (AutonomousConstants.kFarthestAngle - AutonomousConstants.kClosestAngle);
    }

    public void autoaim() {
        int apriltag = Helpers.getAllianceSpeakerApriltag();

        double shooterAngle = calculateShooterAngle(apriltag);
        double robotAngle = calculateDrivebaseAngle(apriltag);

        m_drivetrain.faceHeading(robotAngle);
        m_shooter.setPivot(-shooterAngle);
    }

    public Command autoaimCmd() {
        return runOnce(() ->
            autoaim()
        );
    }
}
 