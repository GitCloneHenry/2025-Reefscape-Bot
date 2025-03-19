package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Neo550;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;

public class FloorIntakeSubsystem extends SubsystemBase {
  private final SparkMax m_floorIntakeDriveMotor =
      new SparkMax(CANConstants.kFloorIntakeDriveMotorID, MotorType.kBrushless);
  private final TalonFX m_floorIntakeAngleMotor =
      new TalonFX(CANConstants.kFloorIntakeAngleMotorID);
  ;

  private final TalonFXConfiguration m_floorIntakeAngleConfiguration = new TalonFXConfiguration();

  private final SparkClosedLoopController m_floorIntakeDriveController =
      m_floorIntakeDriveMotor.getClosedLoopController();

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final DutyCycleEncoder m_floorIntakeAngleEncoder =
      new DutyCycleEncoder(DIOConstants.kFloorIntakeAngleEncoderID);

  public FloorIntakeSubsystem() {
    // waitMillis(2000);
    // applyMotorConfigurations();
  }

  public void waitMillis(double milliseconds) {
    long startTime = System.currentTimeMillis();
    while (System.currentTimeMillis() - startTime < milliseconds) {}
  }

  public void applyMotorConfigurations() {
    Slot0Configs angleSlot0 = m_floorIntakeAngleConfiguration.Slot0;

    MotionMagicConfigs angleMotionMagic = m_floorIntakeAngleConfiguration.MotionMagic;

    angleSlot0.kS = 0.00;
    angleSlot0.kV = 0.00;
    angleSlot0.kP = 0.20;
    angleSlot0.kI = 0.0;
    angleSlot0.kD = 0.0;

    angleMotionMagic.MotionMagicCruiseVelocity = 80;
    angleMotionMagic.MotionMagicAcceleration = 160;
    angleMotionMagic.MotionMagicJerk = 1600;

    m_floorIntakeAngleMotor.getConfigurator().apply(m_floorIntakeAngleConfiguration, 0.050);
    m_floorIntakeDriveMotor.configure(
        Neo550.neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_floorIntakeAngleMotor.setPosition((m_floorIntakeAngleEncoder.get() - 0.304) * 49.5);
    m_floorIntakeAngleMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Command moveToPosition(double value) {
    return Commands.runOnce(
        () -> {
          m_floorIntakeAngleMotor.setControl(m_motionMagicVoltage.withPosition(value));
          // m_targetIntakePosition = Math.min(Math.max(value,
          // EncoderConstants.kMinimumAcceptableFloorIntakePosition),
          // EncoderConstants.kMaximumAcceptableFloorIntakePosition);
        },
        this);
  }

  public Command setDrivePower(double value) {
    return Commands.runOnce(
        () -> {
          m_floorIntakeDriveController.setReference(value, ControlType.kMAXMotionVelocityControl);
        },
        this);
  }

  @Override
  public void periodic() {
    System.err.println(
        (m_floorIntakeAngleEncoder.get() - 0.304) * 49.5
            + ", "
            + m_floorIntakeAngleMotor.getPosition().getValueAsDouble());
    // System.err.println(m_floorIntakeAngleEncoder.get());
  }
}
