package frc.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class Helpers {
    public Helpers() {

    }

    public static void sendSwerveWidget(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight, AHRS gyro) {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> rearLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> rearLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> rearRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> rearRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> gyro.getRotation2d().getRadians(), null);
            }
        });
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
