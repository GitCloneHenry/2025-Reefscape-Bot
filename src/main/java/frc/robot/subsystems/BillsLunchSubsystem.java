package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class BillsLunchSubsystem extends SubsystemBase {
  private final TalonSRX m_lunchDriveMotor = new TalonSRX(CANConstants.kBillsLunchDriveID);

  public BillsLunchSubsystem() {
    applyMotorConfigurations();
  }

  public void applyMotorConfigurations() {
    m_lunchDriveMotor.config_kF(0, 0.000);
    m_lunchDriveMotor.config_kP(0, 15.000);
    m_lunchDriveMotor.config_kI(0, 0.010);
    m_lunchDriveMotor.config_kD(0, 0.001);
    m_lunchDriveMotor.config_IntegralZone(0, 10);

    m_lunchDriveMotor.setNeutralMode(NeutralMode.Coast);
  }

  public Command setSpeed(double speed) {
    return Commands.runOnce(
        () -> {
          m_lunchDriveMotor.set(ControlMode.PercentOutput, speed);
        },
        this);
  }

  public Command stop() {
    return Commands.runOnce(() -> m_lunchDriveMotor.set(ControlMode.PercentOutput, 0));
  }

  @Override
  public void periodic() {}
}
