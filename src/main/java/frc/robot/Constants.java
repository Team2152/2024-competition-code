package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.util.Color;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;



public final class Constants {
  public static final class NoteAlignConstants {
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTolerance = 1;
  }

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
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

  public static class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;

    public static final int kLedPort = 0;
    public static final int kLedLength = 107;

    public static final Color kLedOrange = new Color(255, 50, 0);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
