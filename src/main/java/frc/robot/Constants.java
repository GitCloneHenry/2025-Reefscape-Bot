package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }
  public static class CANConstants {
    public static final int kManipulatorDriveMotorID = 3;
    public static final int kManipulatorAngleMotorID = 1;
    public static final int kElevatorDriveID         = 2;
    public static final int kFloorIntakeDriveMotorID = 3;
    public static final int kFloorIntakeAngleMotorID = 4;
  }
  public static class DIOConstants {
    public static final int kManipulatorAngleEncoderID = 0;
    public static final int kElevatorOpticalSensorID   = 1;
    public static final int kFloorIntakeAngleEncoderID = 2;
  }
  public static class EncoderConstants {
    public static final double kDesiredManipulatorPositionExtended  = 100;
    public static final double kDesiredManipulatorPositionRetracted = 0;
  }
  public static final class ModuleConstants {
    public static final double kFloorIntakeDriveFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
