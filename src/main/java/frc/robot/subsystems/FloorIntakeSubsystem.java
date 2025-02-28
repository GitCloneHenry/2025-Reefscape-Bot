package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Neo550;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;

public class FloorIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_floorIntakeDriveMotor;
    private final TalonFX  m_floorIntakeAngleMotor;

    private final TalonFXConfiguration m_floorIntakeAngleConfiguration = new TalonFXConfiguration();

    // private final RelativeEncoder m_floorIntakeDriveEncoder;

    private final SparkClosedLoopController m_floorIntakeDriveController;

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final DutyCycleEncoder m_floorIntakeAngleEncoder = new DutyCycleEncoder(DIOConstants.kFloorIntakeAngleEncoderID);

    public FloorIntakeSubsystem() {
        m_floorIntakeDriveMotor = new SparkMax(CANConstants.kFloorIntakeDriveMotorID, MotorType.kBrushless);
        m_floorIntakeAngleMotor = new TalonFX (CANConstants.kFloorIntakeAngleMotorID);

        // m_floorIntakeDriveEncoder = m_floorIntakeDriveMotor.getEncoder();

        m_floorIntakeDriveController = m_floorIntakeDriveMotor.getClosedLoopController();

        applyMotorConfigurations();
    }

    public void applyMotorConfigurations() {
        Slot0Configs angleSlot0 = m_floorIntakeAngleConfiguration.Slot0;

        MotionMagicConfigs angleMotionMagic = m_floorIntakeAngleConfiguration.MotionMagic;

        angleSlot0.kS = 0.24;
        angleSlot0.kV = 0.12;
        angleSlot0.kP = 0.11;
        angleSlot0.kI = 0.5;
        angleSlot0.kD = 0.001;

        angleMotionMagic.MotionMagicCruiseVelocity = 80; 
        angleMotionMagic.MotionMagicAcceleration   = 160;
        angleMotionMagic.MotionMagicJerk           = 1600;

        m_floorIntakeAngleMotor.getConfigurator().apply(
            m_floorIntakeAngleConfiguration, 0.050);
        m_floorIntakeDriveMotor.configure(
                Neo550.neoConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
        
        m_floorIntakeAngleMotor.setPosition(m_floorIntakeAngleEncoder.get());
        m_floorIntakeAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command extendAndIntake() {
        return Commands.runOnce(() -> {
            m_floorIntakeAngleMotor.setControl(m_motionMagicVoltage.withPosition(100));
            m_floorIntakeDriveController.setReference(1, ControlType.kVelocity);
        }, this);
    }

    public Command retractAndStop() {
        m_floorIntakeAngleMotor.setControl(m_motionMagicVoltage.withPosition(100));
        return Commands.runOnce(() -> m_floorIntakeDriveController.setReference(0, ControlType.kVelocity), this);
    }
}
