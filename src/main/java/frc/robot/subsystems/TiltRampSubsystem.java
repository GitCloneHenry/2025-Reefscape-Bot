package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.Constants.EncoderConstants;

public class TiltRampSubsystem extends SubsystemBase {
  private final SparkMax m_tiltRampDriveMotor =
      new SparkMax(CANConstants.kTiltRampDriveMotorID, MotorType.kBrushless);
  ;
  private final TalonFXS m_tiltRampAngleMotor = new TalonFXS(CANConstants.kTiltRampAngleMotorID);

  private final TalonFXSConfiguration m_tiltRampAngleConfiguration = new TalonFXSConfiguration();

  private final SparkClosedLoopController m_tiltRampDriveController =
      m_tiltRampDriveMotor.getClosedLoopController();

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final DutyCycleEncoder m_tiltRampAngleEncoder =
      new DutyCycleEncoder(DIOConstants.kTiltRampAngleEncoderID);

  private double m_targetRampPosition = 0.0;

  public TiltRampSubsystem() {
    // waitMillis(3000);
    // applyMotorConfigurations();
  }

  public void waitMillis(double milliseconds) {
    long startTime = System.currentTimeMillis();
    while (System.currentTimeMillis() - startTime < milliseconds) {}
  }

  public void applyMotorConfigurations() {
    Slot0Configs angleSlot0 = m_tiltRampAngleConfiguration.Slot0;

    MotionMagicConfigs angleMotionMagic = m_tiltRampAngleConfiguration.MotionMagic;

    m_tiltRampAngleConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    angleSlot0.kS = 0.0;
    angleSlot0.kV = 0.0;
    angleSlot0.kP = 0.45;
    angleSlot0.kI = 0.0;
    angleSlot0.kD = 0.0;

    angleMotionMagic.MotionMagicCruiseVelocity = 80;
    angleMotionMagic.MotionMagicAcceleration = 160;
    angleMotionMagic.MotionMagicJerk = 1600;

    m_tiltRampAngleMotor.getConfigurator().apply(m_tiltRampAngleConfiguration, 0.050);
    m_tiltRampDriveMotor.configure(
        Neo550.neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_tiltRampAngleMotor.setPosition((m_tiltRampAngleEncoder.get() - 0.6) * 300.0);
    m_tiltRampAngleMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command moveToPossitionCommand(double value) {
    return Commands.runOnce(
        () -> {
          m_targetRampPosition =
              Math.min(
                  Math.max(value, EncoderConstants.kMinimumAcceptableTiltRampPosition),
                  EncoderConstants.kMaximumAcceptableTiltRampPosition);
        },
        this);
  }

  public void moveToPossition(double value) {
    m_targetRampPosition =
        Math.min(
            Math.max(value, EncoderConstants.kMinimumAcceptableTiltRampPosition),
            EncoderConstants.kMaximumAcceptableTiltRampPosition);
  }

  public Command setDrivePowerCommand(double value) {
    return Commands.run(
        () -> {
          m_tiltRampDriveController.setReference(value, ControlType.kMAXMotionVelocityControl);
        },
        this);
  }

  public void setDrivePower(double value) {
    m_tiltRampDriveController.setReference(value, ControlType.kMAXMotionVelocityControl);
  }

  public double getPosition() {
    return m_tiltRampAngleMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    m_tiltRampAngleMotor.setControl(m_motionMagicVoltage.withPosition(m_targetRampPosition));
    // System.err.println(m_tiltRampAngleMotor.getPosition().getValueAsDouble());
  }
  ;
}
