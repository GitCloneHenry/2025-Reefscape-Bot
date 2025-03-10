package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.List;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }

  public static final class CANConstants {
    public static final int kManipulatorDriveMotorID = 3;
    public static final int kManipulatorAngleMotorID = 4;
    public static final int kElevatorDriveID = 1;
    public static final int kFloorIntakeDriveMotorID = 10;
    public static final int kFloorIntakeAngleMotorID = 6;
    public static final int kClimberAngleMotorID = 5;
    public static final int kBillsLunchDriveID = 7;
    public static final int kTiltRampDriveMotorID = 9;
    public static final int kTiltRampAngleMotorID = 2;
  }

  public static final class DIOConstants {
    public static final int kManipulatorAngleEncoderID = 4;
    public static final int kElevatorOpticalSensorID = 0;
    public static final int kFloorIntakeAngleEncoderID = 2;
    public static final int kClimberAngleEncoderID = 3;
    public static final int kTiltRampAngleEncoderID = 1;
  }

  public static final class EncoderConstants {
    public static final double kDesiredManipulatorPositionExtended = 25.034912109375;
    public static final double kDesiredManipulatorPositionRetracted = 0;
    public static final double kMinimumAcceptableClimberPosition = -70.80224609375;
    public static final double kMaximumAcceptableClimberPosition = 80.82275390625;
    public static final double kMinimumAcceptableTiltRampPosition = -75.0;
    public static final double kMaximumAcceptableTiltRampPosition = 0.0;
    public static final double kMinimumAcceptableFloorIntakePosition = -75.0;
    public static final double kMaximumAcceptableFloorIntakePosition = 0.0;
    public static final double kMinimumAcceptableElevatorPosition = -420.0 * 36 / 100;
    public static final double kMaximumAcceptableElevatorPosition = 0.0;
    public static final double kMinimumElevatorHeight = 97.2;
    public static final double kMaxumumElevatorHeight = 199.0;
    public static final double kMaximumAcceptableClosedLoopError = 0.5;
    // L4 Height: 191.8 cm
    // L3 Height: 108.8 cm
    // L1 Height: 112.8 cm
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kSlowModeSpeedPercentage = 0.3;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Translation for each wheel on the bot
    public static final List<Translation2d> wheelTranslations =
        List.of(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            wheelTranslations.get(0),
            wheelTranslations.get(1),
            wheelTranslations.get(2),
            wheelTranslations.get(3));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kFloorIntakeDriveFreeSpeedRps =
        NeoMotorConstants.kFreeSpeedRpm / 60.0;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 30;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 10;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 30;

    // public static final double kPXController = 1;
    // public static final double kPYController = 1;
    // public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-4.0),
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(30.0)),
            new Rotation3d(0.0, 0.0, 0.0));
  }
}
