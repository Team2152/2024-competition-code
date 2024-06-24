package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        
        public static final double kDriveDeadband = 0.15;

        public static final int kSpeedLimiterModifier = 1;

        public static final class LedColors {
            public static final Color kDefault = new Color(255, 50, 0);
            public static final Color kWaiting = new Color(255, 255, 0);
            public static final Color kReady = new Color(0, 255, 0);
        }

        public static final int kLedPwmPort = 0;
        public static final int kLedLength = 108;
    }

    public static final class VisionConstants {
        // Apriltag Layout
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // Camera Profiles
        public static final class RearCamera {
            public static final String kCameraName = "rearCamera";

            // Cam mounted facing forward, half a meter forward of center, half a meter up from center. = new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)
            public static final Transform3d kCameraOffset =
                new Transform3d(new Translation3d(-0.267, 0.0, 0.457), new Rotation3d(0, 0.350, 3.142));

            // The standard deviations of our vision estimated poses, which affect correction rate
            // (Fake values. Experiment and determine estimation noise on an actual robot.)
            public static final Matrix<N3, N1> kSingleTagStdDevs = 
                VecBuilder.fill(4, 4, 8);
            public static final Matrix<N3, N1> kMultiTagStdDevs = 
                VecBuilder.fill(0.5, 0.5, 1);
        }
    }

    public static final class ShooterConstants {
        public static final int kPivotCanId = 15;
        public static final int kLeftShooterCanId = 14;
        public static final int kRightShooterCanId = 13;
        public static final int kLeftFeederCanId = 15;
        public static final int kRightFeederCanId = 16;
    
        public static final class Setpoints {
            public static final double kStow = 5;
            public static final double kHandoff = -28;
            public static final double kFlat = -85;
        }

        public static final class PivotPID {
            public static final double kS = 0.25;
            public static final double kV = 2.0;
            public static final double kA = 0.01;
            public static final double kP = 25.0;
            public static final double kI = 0.0;
            public static final double kD = 0.2;

            public static final double kCruiseVelocity = 300;
            public static final double kAcceleration = 120;
            public static final double kJerk = 300;
        }
    
        public static final double kNegativeShooterLimit = -102;
        public static final double kPositiveShooterLimit = 5;

        public static final double kShooterBias = 0.35;
        public static final double kPivotRatio = 62.91;
    
        public static final double kShooterHeight = Units.inchesToMeters(17);
        public static final boolean kFeederInverted = true;
    
        public static final double kShooterOn = 1;
        public static final double kFeederOn = 1;
    }

    public static final class IntakeConstants {
        public static final int kPivotCanId = 16;
        public static final int kIntakeCanId = 11;
    
        public static final class Setpoints {
            public static final double kIntake = 174;
            public static final double kStow = -10;
            public static final double kAmp = 65;
        }

        public static final class PivotPID {
            public static final double kS = 0.16;
            public static final double kG = 0.64;
            public static final double kV = 0.8;
            public static final double kA = 0.01;
            public static final double kP = 10.0;
            public static final double kI = 0.0;
            public static final double kD = 0.1;

            public static final double kCruiseVelocity = 80;
            public static final double kAcceleration = 160;
            public static final double kJerk = 1600;
        }

        public static final double kPivotRatio = 15;

        public static final boolean kPivotInverted = false;
        public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
        public static final boolean kIntakeInverted = false;
    
        public static final double kIntakeIntake = -1;
        public static final double kIntakeOuttake = 1;
      }

    public static final class DrivetrainConstants {
        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(18);
        
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(25);
        
        // Kinematics
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxAngularSpeed = 2 * Math.PI;

        // Angular Offset of Modules
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
        
        // Driving CAN IDs
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kFrontRightDrivingCanId = 6;
        public static final int kRearRightDrivingCanId = 8;

        // Steering CAN IDs        
        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 4;
    }

    public static final class ModuleConstants {
        // Pinion Gear Tooth Count
        public static final int kDrivingMotorPinionTeeth = 13;

        // Invert the Turning Absolute Encoder
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kMaxSpeedMetersPerSecond = (kDrivingMotorFreeSpeedRps / kDrivingMotorReduction) * kWheelCircumferenceMeters;

        // Drive Motor PID
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        // Steering Motor PID
        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        // Motor Modes
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;
    }

    public static final class AutonomousConstants {
        // The Maximum ALLOWED Speed for Autonomous
        public static final double kMaxSpeedMetersPerSecond = 4.2;

        // PID Configuration for Robot Azimuth Tracking
        public static final double kAzimuthP = 0.01;
        public static final double kAzimuthI = 0;
        public static final double kAzimuthD = 0;

        public static final double kAzimuthTolerance = 3;

        // Auto Aim
        public static final double kClosestDistance = 2; // Closest distance we can shoot into the speaker.
        public static final double kFarthestDistance = 3.2258; // Farthest distance we can shoot into the speaker.
    
        public static final double kClosestAngle = 32; // Angle used for the closest distance.
        public static final double kFarthestAngle = 47.34; // Angle used for the farthest distance.
    }

    public static final class AmpLimits {
        // Amp Limit for Swerve Drive Motor
        public static final int kSwerveDrive = 80;

        // Amp Limit for Swerve Steering Motor
        public static final int kSwerveSteer = 40;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
