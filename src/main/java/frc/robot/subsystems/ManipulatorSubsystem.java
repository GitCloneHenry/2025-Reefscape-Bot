package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final TalonFXS m_manipulatorDriveMotor = new TalonFXS(CANConstants.kManipulatorDriveMotorID);
    private final TalonFX  m_manipulatorAngleMotor = new TalonFX (CANConstants.kManipulatorAngleMotorID);

    private final TalonFXSConfiguration m_manipulatorDriveConfiguration = new TalonFXSConfiguration();
    private final TalonFXConfiguration  m_manipulatorAngleConfiguration = new  TalonFXConfiguration();

    // private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final DutyCycleEncoder m_manipulatorAngleEncoder = new DutyCycleEncoder(DIOConstants.kManipulatorAngleEncoderID);

    public ManipulatorSubsystem() {
        applyMotorConfigurations();
    }

    /**Creates and Applies Motor Configurations */
    public void applyMotorConfigurations() {
        Slot0Configs driveSlot0 = m_manipulatorDriveConfiguration.Slot0;
        Slot0Configs angleSlot0 = m_manipulatorAngleConfiguration.Slot0;

        MotionMagicConfigs angleMotionMagic = m_manipulatorAngleConfiguration.MotionMagic;

        driveSlot0.kS = 0.24;
        driveSlot0.kV = 0.12;
        driveSlot0.kP = 0.11;
        driveSlot0.kI = 0.5;
        driveSlot0.kD = 0.001;

        angleSlot0.kS = 0.24;
        angleSlot0.kV = 0.12;
        angleSlot0.kP = 0.11;
        angleSlot0.kI = 0.5;
        angleSlot0.kD = 0.001;

        angleMotionMagic.MotionMagicCruiseVelocity = 80; 
        angleMotionMagic.MotionMagicAcceleration   = 160;
        angleMotionMagic.MotionMagicJerk           = 1600;

        m_manipulatorDriveConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        m_manipulatorDriveMotor.getConfigurator().apply(
            m_manipulatorDriveConfiguration, 0.050);
        m_manipulatorAngleMotor.getConfigurator().apply(
                m_manipulatorAngleConfiguration, 0.050);

        m_manipulatorDriveMotor.setNeutralMode(NeutralModeValue.Coast);

        m_manipulatorAngleMotor.setPosition(m_manipulatorAngleEncoder.get());
        m_manipulatorAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**Get Manipulator Command
     * @param speed percentage value [-1, 1] used to control the manipulator's
     * drive motor
     * @return Returns a new RunCommand used to set the speed percentage 
     * for the manipulator's drive motor.
     */
    public Command getManipulatorDriveCommand(double speed) {
        DutyCycleOut manipulatorSpeed = new DutyCycleOut(speed);
        
        return new RunCommand(
            () -> m_manipulatorDriveMotor.setControl(manipulatorSpeed));
        
        // return new RunCommand(() -> m_manipulatorDriveMotor.setControl(m_velocityVoltage.withVelocity(speed)), this);
    }

    /**Extends the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to extend.
     */
    public Command extendCoralManipulator() {
        return Commands.runOnce(() -> System.out.println("mfgh~") /*m_manipulatorAngleMotor.setControl(m_motionMagicVoltage.withPosition(EncoderConstants.kDesiredManipulatorPositionExtended))*/, this);
    }

    /**Retracts the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to retract.
     */
    public Command retractCoralManipulator() {
        return Commands.runOnce(() -> System.out.println("mfgh~") /*m_manipulatorAngleMotor.setControl(m_motionMagicVoltage.withPosition(EncoderConstants.kDesiredManipulatorPositionRetracted))*/, this);
    }

    @Override
    public void periodic() {}
}
