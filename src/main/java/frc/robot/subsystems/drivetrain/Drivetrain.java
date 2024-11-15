package frc.robot.subsystems.drivetrain;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drivetrain.vision.Vision;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase {
    private double maximumAccel;
    private File configDirectory;

    public SwerveDrive m_swerveDrive;
    
    private Vision m_vision;

    public Drivetrain() {
        maximumAccel = Units.feetToMeters(4.5);
        configDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

        try
        {
            m_swerveDrive = new SwerveParser(configDirectory).createSwerveDrive(maximumAccel);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.MACHINE;

        m_swerveDrive.setHeadingCorrection(false);
        m_swerveDrive.setCosineCompensator(false);
        m_swerveDrive.setAngularVelocityCompensation(false, false, 0.1); // Use for Skewed Driving
        m_swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        setupVision();
        setupPathPlanner();
    }
    
    @Override
    public void periodic()
    {
        m_swerveDrive.updateOdometry();
        m_vision.updatePoseEstimation(m_swerveDrive);
    }   

    /*
     * Setup Methods
     */
    public void setupVision() {
        m_swerveDrive.stopOdometryThread();
        m_vision = new Vision(m_swerveDrive::getPose, m_swerveDrive.field);
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                AutoConstants.TRANSLATION_PID,
                AutoConstants.ANGLE_PID,
                maximumAccel,
                m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()
            ),
            () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this);
    }

    /*
     * Swerve Methods
     */
    public Pose2d getPose()
    {
        return m_swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose)
    {
      m_swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return m_swerveDrive.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
      m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void zeroGyro()
    {
        m_swerveDrive.zeroGyro();
    }

    /*
     * Drive Methods
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY, BooleanSupplier speedLimiter)
    {
        //m_swerveDrive.setHeadingCorrection(false); // Normally you would want heading correction for this kind of control.
        return run(() -> {
            double scaleFactor = 0.8;
            if (speedLimiter.getAsBoolean()) {
                //scaleFactor /= DriveConstants.kLimiterModifier;
            }

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), scaleFactor);
            ChassisSpeeds speeds = m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                headingX.getAsDouble(),
                headingY.getAsDouble(),            
                m_swerveDrive.getOdometryHeading().getRadians(),
                m_swerveDrive.getMaximumVelocity());

            if (false) {
                driveRobotOriented(speeds);
            } else {
                driveFieldOriented(speeds);
            }
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    public void driveRobotOriented(ChassisSpeeds velocity)
    {
        m_swerveDrive.drive(velocity);
    }

    /*
     * PathPlanner Methods
     */
    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }
}
