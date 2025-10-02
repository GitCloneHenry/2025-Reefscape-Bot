package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorDrivePrimary = new TalonFX(CANConstants.kElevatorDriveID);
  private final TalonFX m_elevatorDriveSecondary = new TalonFX(CANConstants.kElevatorFollowerID);

  private final Follower m_elevatorDriveFollower = new Follower(
    m_elevatorDrivePrimary.getDeviceID(),
    false
  );

  private final TalonFXConfiguration m_elevatorDriveConfiguration = new TalonFXConfiguration();

  private final DigitalInput m_opticalSensor =
      new DigitalInput(DIOConstants.kElevatorOpticalSensorID);

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0);

  private final double[] m_elevatorIncrements = {110.0, 100.0, 115.0, 190.0, 200.0};

  private int m_positionPointer = 0;

  public ElevatorSubsystem() {
    applyMotorConfigurations();
  }

  public TalonFX getDriveMotor() {
    return m_elevatorDrive;
  }

  public void applyMotorConfigurations() {
    Slot0Configs elevatorSlot0 = m_elevatorDriveConfiguration.Slot0;

    MotionMagicConfigs elevatorMotionMagic = m_elevatorDriveConfiguration.MotionMagic;
    AudioConfigs audioConfigs = m_elevatorDriveConfiguration.Audio;

    audioConfigs.AllowMusicDurDisable = true;

    elevatorSlot0.kS = 0.00;
    elevatorSlot0.kV = 0.00;
    elevatorSlot0.kP = 0.90;
    elevatorSlot0.kI = 0.00;
    elevatorSlot0.kD = 0.00;

    elevatorMotionMagic.MotionMagicCruiseVelocity = 6000;
    elevatorMotionMagic.MotionMagicAcceleration = 12000;
    elevatorMotionMagic.MotionMagicJerk = 24000;

    m_elevatorDrivePrimary.getConfigurator().apply(m_elevatorDriveConfiguration, 0.050);
    m_elevatorDriveSecondary.getConfigurator().apply(m_elevatorDriveConfiguration, 0.050);

    m_elevatorDrivePrimary.setNeutralMode(NeutralModeValue.Brake);
    m_elevatorDriveSecondary.setNeutralMode(NeutralModeValue.Brake);

    m_elevatorDriveSecondary.setControl(m_elevatorDriveFollower);
  }

  public boolean isSensorTriggered() {
    return m_opticalSensor.get();
  }

  public void moveElevator(double speed) {
    DutyCycleOut elevatorDriveSpeed = new DutyCycleOut(speed);

    m_elevatorDrive.setControl(elevatorDriveSpeed);
  }

  public void moveElevatorToPosition(double position) {
    m_elevatorDrive.setControl(
        m_motionMagicVoltage.withPosition(
            Math.max(
                Math.min(position, EncoderConstants.kMaximumAcceptableElevatorPosition),
                EncoderConstants.kMinimumAcceptableElevatorPosition)));
  }

  public Command moveElevatorToPositionCommand(double position) {
    return Commands.runOnce(() -> moveElevatorToPosition(position), this);
  }

  public Command moveElevatorToHeightCommand(double height) {
    return Commands.runOnce(
        () ->
            moveElevatorToPosition(
                (height - EncoderConstants.kMinimumElevatorHeight)
                    / (EncoderConstants.kMaxumumElevatorHeight
                        - EncoderConstants.kMinimumElevatorHeight)
                    * EncoderConstants.kMinimumAcceptableElevatorPosition),
        this);
  }

  public void applyElevatorSpeed(double speed) {
    m_elevatorDrive.set(speed);
  }

  public void resetEncoder() {
    m_elevatorDrive.setPosition(0);
  }

  public double getEncoderPosition() {
    return m_elevatorDrive.getPosition().getValueAsDouble();
  }

  public void incrementElevatorPosition() {
    m_positionPointer = Math.min(m_positionPointer + 1, 4);
    moveElevatorToPosition(
        (m_elevatorIncrements[m_positionPointer] - EncoderConstants.kMinimumElevatorHeight)
            / (EncoderConstants.kMaxumumElevatorHeight - EncoderConstants.kMinimumElevatorHeight)
            * EncoderConstants.kMinimumAcceptableElevatorPosition);
  }

  public void decrementElevatorPosition() {
    m_positionPointer = Math.max(m_positionPointer - 1, 0);
    moveElevatorToPosition(
        (m_elevatorIncrements[m_positionPointer] - EncoderConstants.kMinimumElevatorHeight)
            / (EncoderConstants.kMaxumumElevatorHeight - EncoderConstants.kMinimumElevatorHeight)
            * EncoderConstants.kMinimumAcceptableElevatorPosition);
  }

  public void incrementElevatorToLevel(int positionPointer) {
    moveElevatorToPosition(
      (m_elevatorIncrements[Math.max(Math.min(positionPointer, 3), 0)] - EncoderConstants.kMinimumElevatorHeight)
          / (EncoderConstants.kMaxumumElevatorHeight - EncoderConstants.kMinimumElevatorHeight)
          * EncoderConstants.kMinimumAcceptableElevatorPosition);
  }

  public double getErrorFromTarget() {
    return Math.abs(m_elevatorDrive.getClosedLoopError().getValueAsDouble());
  }

  public void resetIncrements() {
    m_positionPointer = -1;
  }
  

  @Override
  public void periodic() {
    // System.err.println(m_elevatorDrive.getPosition().getValueAsDouble());
    // m_elevatorDrive.set(-0.08);
  }
}
