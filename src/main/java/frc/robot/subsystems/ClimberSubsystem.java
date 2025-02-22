package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX m_manipulatorAngleMotor = new TalonFX(CANConstants.kManipulatorAngleMotorID);

    private final TalonFXConfiguration m_manipulatorAngleConfiguration = new TalonFXConfiguration();

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final DutyCycleEncoder m_manipulatorAngleEncoder = new DutyCycleEncoder(DIOConstants.kManipulatorAngleEncoderID);

    private double minimumClimberPosition = 0;
    private double maximumClimberPosition = 100;

    private double m_targetClimberPosition;

    public ClimberSubsystem(CommandXboxController driverController) {
        applyMotorConfigurations(); 
    }

    public void applyMotorConfigurations() {
        Slot0Configs angleSlot0 = m_manipulatorAngleConfiguration.Slot0;

        MotionMagicConfigs angleMotionMagic = m_manipulatorAngleConfiguration.MotionMagic;

        angleSlot0.kS = 0.24;
        angleSlot0.kV = 0.12;
        angleSlot0.kP = 0.11;
        angleSlot0.kI = 0.5;
        angleSlot0.kD = 0.001;

        angleMotionMagic.MotionMagicCruiseVelocity = 80; 
        angleMotionMagic.MotionMagicAcceleration   = 160;
        angleMotionMagic.MotionMagicJerk           = 1600;

        m_manipulatorAngleMotor.getConfigurator().apply(
                m_manipulatorAngleConfiguration, 0.050);
        
        m_manipulatorAngleMotor.setPosition(m_manipulatorAngleEncoder.get());
        m_manipulatorAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void incrementClimberPosition(double value) {
        double encoderPosition = m_manipulatorAngleMotor.getPosition().getValueAsDouble();

        if (encoderPosition + value < minimumClimberPosition || encoderPosition + value > maximumClimberPosition) {
            return;
        }

        m_targetClimberPosition += value;
    }

    @Override
    public void periodic() {
        m_manipulatorAngleMotor.setControl(m_motionMagicVoltage.withPosition(m_targetClimberPosition));
    }
}
