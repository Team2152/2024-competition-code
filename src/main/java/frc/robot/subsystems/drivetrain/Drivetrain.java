
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.vision.Limelight;
import frc.utils.Helpers;
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

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
    DrivetrainConstants.kFrontLeftDrivingCanId,
    DrivetrainConstants.kFrontLeftTurningCanId,
    DrivetrainConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    DrivetrainConstants.kFrontRightDrivingCanId,
    DrivetrainConstants.kFrontRightTurningCanId,
    DrivetrainConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
    DrivetrainConstants.kRearLeftDrivingCanId,
    DrivetrainConstants.kRearLeftTurningCanId,
    DrivetrainConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
    DrivetrainConstants.kRearRightDrivingCanId,
    DrivetrainConstants.kRearRightTurningCanId,
    DrivetrainConstants.kBackRightChassisAngularOffset);


  private AHRS m_gyro;
  private Field2d m_field;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Limelight m_rearCamera;
  private Pose2d lastPose;

  private PIDController rotationController;
  private boolean headingLocked;
  private double headingTarget;
  private double rotationOffset;

  public Drivetrain(Limelight m_rearCamera) {
      AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
          new PIDConstants(5, 0, 0),
          new PIDConstants(5, 0, 0),
          AutonomousConstants.kMaxSpeedMetersPerSecond,
          ((DrivetrainConstants.kWheelBase > DrivetrainConstants.kTrackWidth) ? DrivetrainConstants.kWheelBase : DrivetrainConstants.kTrackWidth) / 2,
          new ReplanningConfig()
        ),
        () -> Helpers.isRedAlliance(),
        this
      );

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DrivetrainConstants.kDriveKinematics, 
      getHeading(), 
      getModulePositions(), 
      getPose()
    );

    SmartDashboard.putData("Field", m_field);
    Helpers.sendSwerveWidget(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, m_gyro);

    m_gyro = new AHRS(SPI.Port.kMXP);
    m_field = new Field2d();

    rotationController = new PIDController(AutonomousConstants.kAzimuthP, AutonomousConstants.kAzimuthI, AutonomousConstants.kAzimuthD);
    rotationController.setTolerance(AutonomousConstants.kAzimuthTolerance);
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

  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

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

  public void drive(double xSpeed, double ySpeed, double rot, boolean limiterEnabled) {
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double rotDelivered = rot + rotationOffset;
    
    if (limiterEnabled) {
        xSpeedCommanded /= OIConstants.kSpeedLimiterModifier;
        ySpeedCommanded /= OIConstants.kSpeedLimiterModifier;
        rotDelivered /= OIConstants.kSpeedLimiterModifier;
    }

    xSpeedCommanded *= ModuleConstants.kMaxSpeedMetersPerSecond;
    ySpeedCommanded *= ModuleConstants.kMaxSpeedMetersPerSecond;
    rotDelivered *= DrivetrainConstants.kMaxAngularSpeed;

    var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotDelivered, getHeading()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, ModuleConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command teleopDrive(
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier speedLimiter) {
      return run(() -> {
          double xVal = -MathUtil.applyDeadband(x.getAsDouble(), OIConstants.kDriveDeadband);
          double yVal = -MathUtil.applyDeadband(y.getAsDouble(), OIConstants.kDriveDeadband);
          double rotVal = -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDriveDeadband);

          boolean speedLimiterVal = speedLimiter.getAsBoolean();

          drive(xVal, yVal, rotVal, speedLimiterVal);
      });
  }

  public Command setX() {
    return run(() -> {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    });
  }

  private void setHeadingLock(double heading) {
    headingLocked = true;
    headingTarget = heading;
  }

  public Command faceHeadingCommand(double heading) {
    return run(() -> {
      setHeadingLock(heading);
    });
  }

  public void faceHeading(double heading) {
    setHeadingLock(heading);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ModuleConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public Command zeroHeadingCmd() {
    return runOnce(() -> {m_gyro.reset();});
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }
}