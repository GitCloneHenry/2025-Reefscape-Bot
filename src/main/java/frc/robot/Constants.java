package frc.robot;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }
  public static final class CANConstants {
    public static final int kManipulatorDriveMotorID = 3;
    public static final int kManipulatorAngleMotorID = 4;
    public static final int kElevatorDriveID         = 2;
    public static final int kFloorIntakeDriveMotorID = 3;
    public static final int kFloorIntakeAngleMotorID = 4;
    public static final int kClimberAngleMotorID     = 5;
    public static final int kBillsLunchDriveID       = 7;
    public static final int kTiltRampDriveMotorID    = 8;
    public static final int kTiltRampAngleMotorID    = 9;
  }
  public static final class DIOConstants {
    public static final int kManipulatorAngleEncoderID = 4;
    public static final int kElevatorOpticalSensorID   = 1;
    public static final int kFloorIntakeAngleEncoderID = 2;
    public static final int kClimberAngleEncoderID     = 3;
    public static final int kTiltRampAngleEncoderID    = 0;
  }
  public static final class EncoderConstants {
    public static final double kDesiredManipulatorPositionExtended  = 25.034912109375 + 1.3;
    public static final double kDesiredManipulatorPositionRetracted = 0;
    public static final double kMinimumAcceptableClimberPosition    = -54.80224609375;
    public static final double kMaximumAcceptableClimberPosition    =  80.82275390625;
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class ModuleConstants {
    public static final double kFloorIntakeDriveFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
  }
}
