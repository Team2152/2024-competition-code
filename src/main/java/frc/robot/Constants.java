package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkBase.IdleMode;



public final class Constants {
  public static final class NoteAlignConstants {
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTolerance = 1;
  }

  public static final class HeadingAlignConstants {
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTolerance = 5;
  }

  public static final class AutoAimConstants {
    // Must be in meters
    public static final double kClosestDistance = 2; // 9144    // Closest distance we can shoot into the speaker.
    public static final double kFarthestDistance = 3.2258;  // Farthest distance we can shoot into the speaker.

    public static final double kClosestAngle = 32;      // Angle used for the closest distance.
    public static final double kFarthestAngle = 47.34;     // Angle used for the farthest distance.
  }

  public static final class IntakeConstants {
    public static final int kPivotCanId = 16;
    public static final int kIntakeCanId = 11;

    public static final boolean kPivotInverted = false;
    public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
    public static final boolean kIntakeInverted = false;
    
    // Amp Limit
    public static final int kIntakeAmps = 50;

    public static final int kIntakeNoteLsPort = 1;

    public static final double kIntakeSetpointHang = 45;
    public static final double kIntakeSetpointOut = 174;
    public static final double kIntakeSetpointIn = -10;

    public static final double kIntakeSetpointAmp = 65;

    public static final double kIntakeIntake = -1;
    public static final double kIntakeOuttake = 1;
  }

  public static final class ShooterConstants {
    public static final int kPivotCanId = 15;
    public static final int kLeftShooterCanId = 14;
    public static final int kRightShooterCanId = 13;
    public static final int kLeftFeederCanId = 15;
    public static final int kRightFeederCanId = 16;

    public static final double kShooterSetpointStow = 5;
    public static final double kShooterSetpointSpeakerDefault = -28;
    public static final double kShooterSetpointFlat = -85;
    public static final double kShooterSetpointHang = -102;

    public static final double kNegativeShooterLimit = -102;
    public static final double kPositiveShooterLimit = 5;

    public static final double kShooterHeight = 0.4318;
    public static final boolean kFeederInverted = true;

    public static final double kShooterOn = 1;
    public static final double kFeederOn = 1;
  }

  public static class Vision {
    public static final String kRearCameraName = "rearCamera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center. = new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)
    public static final Transform3d kRearRobotToCam =
      new Transform3d(new Translation3d(0.2286, 0.0, 0.4953), new Rotation3d(0, 0.314159, Math.PI));//(0, 0.350, 3.142));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kLimiterModifier = 5.0; // speed / modifier

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 80; // amps
    public static final int kTurningMotorCurrentLimit = 30; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;

    public static final int kLedPort = 0;
    public static final int kLedLength = 107;

    public static final Color kLedOrange = new Color(255, 50, 0);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6.94;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
