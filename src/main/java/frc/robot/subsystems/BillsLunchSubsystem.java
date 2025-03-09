package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class BillsLunchSubsystem extends SubsystemBase {
  private TalonSRX m_lunchDriveMotor = new TalonSRX(CANConstants.kBillsLunchDriveID);
  private final double ENCODER_TICKS_PER_REVOLUTION = 44.4;

  public BillsLunchSubsystem() {
    applyMotorConfigurations();
    resetEncoder();
  }

  public void applyMotorConfigurations() {
    m_lunchDriveMotor.config_kF(0, 0.000);
    m_lunchDriveMotor.config_kP(0, 15.000);
    m_lunchDriveMotor.config_kI(0, 0.010);
    m_lunchDriveMotor.config_kD(0, 0.001);
    m_lunchDriveMotor.config_IntegralZone(0, 10);

    m_lunchDriveMotor.setSensorPhase(false);
    m_lunchDriveMotor.setSelectedSensorPosition(0);

    m_lunchDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);

    m_lunchDriveMotor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30);
    m_lunchDriveMotor.overrideLimitSwitchesEnable(true);

    m_lunchDriveMotor.setNeutralMode(NeutralMode.Coast);
  }

  public Command setSpeed(double speed) {
    return Commands.run(
        () -> {
          m_lunchDriveMotor.set(ControlMode.PercentOutput, getEncoderPressed() ? 0 : speed);
        },
        this);
  }

  public void setPosition(double position) {
    m_lunchDriveMotor.set(ControlMode.Position, position);
  }

  public Command moveToPosition(double position) {
    return Commands.runOnce(
        () -> {
          // System.err.println(getEncoderPosition());
          m_lunchDriveMotor.set(ControlMode.Position, position);
        });
  }

  public Command stop() {
    return Commands.runOnce(() -> m_lunchDriveMotor.set(ControlMode.PercentOutput, 0));
  }

  public void resetEncoder() {
    m_lunchDriveMotor.setSelectedSensorPosition(0, 0, 30);
  }

  public double getEncoderPosition() {
    return m_lunchDriveMotor.getSelectedSensorPosition(0);
  }

  public double getPositionInRevolutions() {
    return (double) getEncoderPosition() / ENCODER_TICKS_PER_REVOLUTION;
  }

  public boolean getEncoderPressed() {
    return m_lunchDriveMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // System.err.println(getEncoderPosition());
  }
}
