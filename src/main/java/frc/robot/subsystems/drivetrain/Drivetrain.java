
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HeadingAlignConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NoteTracking;
import frc.robot.subsystems.shooter.Shooter;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);


  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private SwerveDrivePoseEstimator m_poseEstimator;
  private Pose2d lastPose = null;

  private final Field2d m_field = new Field2d();
  
  private final Limelight m_rearCamera;
  private final NoteTracking m_noteTracking;

  private PIDController rotationController;
  private boolean headingLocked;
  private double headingTarget;
  private double rotationOffset;

  public Drivetrain(Limelight camera, NoteTracking noteTracking) {
    m_rearCamera = camera;
    m_noteTracking = noteTracking;

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0),
        DriveConstants.kMaxSpeedMetersPerSecond,
        (DriveConstants.kWheelBase / 39.37 ) / 2,
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
    
    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d()
    );

    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_rearRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_rearRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> m_gyro.getRotation2d().getRadians(), null);
      }
    });

    rotationController = new PIDController(HeadingAlignConstants.kP, HeadingAlignConstants.kI, HeadingAlignConstants.kD);
    rotationController.setTolerance(2);
    headingLocked = false;
    headingTarget = 0;
    rotationOffset = 0;
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public PathPlannerPath createOnTheFlyPath(List<Pose2d> points, double endRotation) {
     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      points
    );

    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(endRotation))
    );
    path.preventFlipping = true;
    return path;
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }


  @Override
  public void periodic() {
    m_poseEstimator.update(getHeading(), getModulePositions());

    Optional<EstimatedRobotPose> estRobotPose = m_rearCamera.getEstimatedPose();
    if (estRobotPose.isPresent()) {
      EstimatedRobotPose estRobotPoseData = estRobotPose.get();
      Matrix<N3, N1> estStdDevs = m_rearCamera.getEstimationStdDevs(estRobotPose.get().estimatedPose.toPose2d());
      Pose2d pose = estRobotPoseData.estimatedPose.toPose2d();
      double timestamp = estRobotPoseData.timestampSeconds;

      if (pose != lastPose) {
        m_poseEstimator.addVisionMeasurement(
          pose,
          timestamp,
          estStdDevs
        );
      }
    }
  

    m_field.setRobotPose(getPose());
    
    if (headingLocked) {
      rotationOffset = rotationController.calculate(getHeading().getDegrees(), headingTarget);
      if (rotationController.atSetpoint()) {
        rotationOffset = 0;
        headingLocked = false;
      }
    } else {
      rotationOffset = m_noteTracking.correction;
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }


  /**
   * Returns the current chassis speed of the robot.
   *
   * @return The chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Changes the current chassis speed of the robot.
   *
   * @param speeds Sets the chassis speed.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   * @param limiterEnabled Whether to limit the driving speed or not
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean limiterEnabled, Rotation2d heading) {
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double rotDelivered = rot + rotationOffset;

    if (limiterEnabled) {
        xSpeedCommanded /= DriveConstants.kLimiterModifier;
        ySpeedCommanded /= DriveConstants.kLimiterModifier;
        rotDelivered /= DriveConstants.kLimiterModifier;
    }

    // Final speed calculations
    xSpeedCommanded *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeedCommanded *= DriveConstants.kMaxSpeedMetersPerSecond;
    rotDelivered *= DriveConstants.kMaxAngularSpeed;

    // Set module states
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotDelivered, heading));
    SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public Command setX() {
    return run(() -> {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    });
  }

  /** 
   * Rotates the chassis until the meeting a requested gyro angle.
  */
  public Command faceHeading(double heading) {
    return run(() -> {
      headingLocked = true;
      headingTarget = heading;
    });
  }

  /** 
   * Rotates the chassis until the meeting a requested gyro angle.
  */
  public void faceHeadingManual(double heading) {
    headingLocked = true;
    headingTarget = heading;
  }

  public Command teleopDrive(
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot,
    BooleanSupplier speedLimiter) {
      return run(() -> {
          double xVal = -MathUtil.applyDeadband(x.getAsDouble(), OIConstants.kDriveDeadband);
          double yVal = -MathUtil.applyDeadband(y.getAsDouble(), OIConstants.kDriveDeadband);
          double rotVal = -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDriveDeadband);

          boolean speedLimiterVal = speedLimiter.getAsBoolean();

          if (m_noteTracking.active) {
            drive(xVal, yVal, rotVal, speedLimiterVal, new Rotation2d(0));
          } else {
            drive(xVal, yVal, rotVal, speedLimiterVal, getHeading());
          }
      });
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return run(() -> {m_gyro.reset();});
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngleToSpeakerApriltag(int apriltagId, double heightOffset, Shooter m_shooter) {
      Pose3d requestedTagPose = Constants.Vision.kTagLayout.getTagPose(apriltagId).get();
      Pose2d currentPose = getPose();
        // tag 7 = blue speaker
        // tag 4 = red speaker 
        // tag 3 = testing red speaker
          // double headingToTarget = Math.atan2(
          //     requestedTagPose.getY() - currentPose.getY(),
          //     requestedTagPose.getX() - currentPose.getX()
          // );

          // double angleToTarget = Math.atan2(
          //     requestedTagPose.getZ() - ShooterConstants.kShooterHeight,
          //     requestedTagPose.getX() - currentPose.getX()
          // );

      double distanceToTarget = Math.sqrt(
        Math.pow(currentPose.getX() - requestedTagPose.getX(), 2) +
        Math.pow(currentPose.getY() - requestedTagPose.getY(), 2) +
        Math.pow(ShooterConstants.kShooterHeight - requestedTagPose.getZ(), 2)
      );

      if (distanceToTarget < AutoAimConstants.kClosestDistance) {
        distanceToTarget = AutoAimConstants.kClosestDistance;
      } else if (distanceToTarget > AutoAimConstants.kFarthestDistance) {
        distanceToTarget = AutoAimConstants.kFarthestDistance;
      }

      double factor = (distanceToTarget - AutoAimConstants.kClosestDistance) / (AutoAimConstants.kFarthestDistance - AutoAimConstants.kClosestDistance);
      double angleRequired = AutoAimConstants.kClosestAngle + factor * (AutoAimConstants.kFarthestAngle - AutoAimConstants.kClosestAngle);

      return angleRequired;
  }

  public double getHeadingFromApriltag(int apriltagId, Pose2d currentPose) {
    Pose3d requestedTagPose = Constants.Vision.kTagLayout.getTagPose(apriltagId).get();

    double headingToTarget = Math.atan2(
      requestedTagPose.getY() - currentPose.getY(),
      requestedTagPose.getX() - currentPose.getX()
    );

    return Units.radiansToDegrees(headingToTarget);
  }
}
