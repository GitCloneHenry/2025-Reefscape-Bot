package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    // Defines a TalonFXS motor which is connected to a Minion.
    private final TalonFXS m_manipulatorDriveMotor = new TalonFXS(CANConstants.kManipulatorDriveMotorID);
    // Defines a TalonFX motor which is connected to a Falcon 500.
    private final TalonFX  m_manipulatorAngleMotor = new TalonFX(CANConstants.kManipulatorAngleMotorID);

    // Defines a configuration for the TalonFXS motor.
    private final TalonFXSConfiguration m_manipulatorDriveConfiguration = new TalonFXSConfiguration();
    // Defines a configuration for the TalonFX motor.
    private final TalonFXConfiguration m_manipulatorAngleConfiguration = new TalonFXConfiguration();

    // Defines a MotionMagicVoltage to ellagantly control the TalonFX.
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    // Defines a DutyCycleEncoder which is a REV Through Bore Encoder.
    private final DutyCycleEncoder m_manipulatorAngleEncoder = new DutyCycleEncoder(DIOConstants.kManipulatorAngleEncoderID);

    private double m_desiredPosition = 0.0;

    private final double[] m_manipulatorIncrements = { 1, 1, 1 };

    private int m_positionPointer = 0;

    /**Creates a Manipulator Subsystem
     * This is used to control the coral manipulator.
     */
    public ManipulatorSubsystem() { 
        // waitMillis(2000);
        // // Applies motor configurations.
        // applyMotorConfigurations();

        SmartDashboard.putNumber("FF", 0);
    }

    public void waitMillis(double milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {}
    }

    /**Creates and Applies Motor Configurations
     * This creates and applies the motor configurations for the 
     * manipulator's drive motor and its turn motor. Along with 
     * this, it creates and configures parameters to be used with
     * motion magic.
     */
    public void applyMotorConfigurations() {
        // Defines a configuration slot for the TalonFX and TalonFXS
        Slot0Configs driveSlot0 = m_manipulatorDriveConfiguration.Slot0;
        Slot0Configs angleSlot0 = m_manipulatorAngleConfiguration.Slot0;

        // Defines a configuration for the MotionMagic
        MotionMagicConfigs angleMotionMagic = m_manipulatorAngleConfiguration.MotionMagic;
        // CurrentLimitsConfigs currentLimitsConfigs = m_manipulatorDriveConfiguration.CurrentLimits;
        // HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = m_manipulatorAngleConfiguration.HardwareLimitSwitch;

        // Tells the TalonFXS that it's connected to a Minion motor.
        m_manipulatorDriveConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // currentLimitsConfigs.StatorCurrentLimit = 10;
        // currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
        // hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

        driveSlot0.kS = 0.24;  // Static Feedforward value for TalonFXS
        driveSlot0.kV = 0.12;  // Velocity Feedforward value for TalonFXS
        driveSlot0.kP = 0.11;  // Proportion value for TalonFXS
        driveSlot0.kI = 0.5;   // Integral value for TalonFXS
        driveSlot0.kD = 0.001; // Derivative value for TalonFXS

        angleSlot0.kS = 0.10;    // Static Feedforward value for TalonFX
        angleSlot0.kV = 0.10;    // Velocity Feedforward value for TalonFX
        angleSlot0.kP = 1.05;    // Proportion value for TalonFX
        angleSlot0.kI = 0.00005; // Integral value for TalonFX
        angleSlot0.kD = 0.0;     // Derivative value for TalonFX

        angleMotionMagic.MotionMagicCruiseVelocity = 110;  // Maximum MotionMagic Velocity
        angleMotionMagic.MotionMagicAcceleration   = 190;  // Maximum MotionMagic Acceleration
        angleMotionMagic.MotionMagicJerk           = 800; // Maximum MotionMagic Jerk

        // Apply configurations for the TalonFXS and TalonFX respectively 
        m_manipulatorDriveMotor.getConfigurator().apply(
            m_manipulatorDriveConfiguration, 0.050);
        m_manipulatorAngleMotor.getConfigurator().apply(
                m_manipulatorAngleConfiguration, 0.050);
        
        // Configure the TalonFXS to coast when no output is specified
        m_manipulatorDriveMotor.setNeutralMode(NeutralModeValue.Coast);

        // Configure the TalonFX's encoder to the position read by the absolute encoder.
        m_manipulatorAngleMotor.setPosition((m_manipulatorAngleEncoder.get() - 0.205810546875) * 100);
        // Configure the TalonFX to brake when no output is specified 
        m_manipulatorAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**Get Manipulator Command
     * @param speed percentage value [-1, 1] used to control the manipulator's
     * drive motor
     * @return Returns a new RunCommand used to set the speed percentage 
     * for the manipulator's drive motor.
     */
    public Command getManipulatorDriveCommand(double speed) {
        // Define a DutyCycleOut that will be used to move the TalonFXS at the specified speed
        DutyCycleOut manipulatorSpeed = new DutyCycleOut(speed);
        
        // Define a RunCommand that is used to set the speed of the motor
        return new RunCommand(
            () -> m_manipulatorDriveMotor.setControl(manipulatorSpeed), this);
    }

    public void setSpeed(double speed) {
        DutyCycleOut manipulatorSpeed = new DutyCycleOut(speed);
        m_manipulatorDriveMotor.setControl(manipulatorSpeed);
    }

    /**Extends the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to extend.
     */
    public Command extendCoralManipulator() {
        // Define a new command that moves the TalonFX to the desired extension
        return new RunCommand(() -> 
            {m_desiredPosition = EncoderConstants.kDesiredManipulatorPositionExtended + 1.3;}  
        /*m_manipulatorAngleMotor.setControl(
                m_motionMagicVoltage.withPosition(
                    EncoderConstants.kDesiredManipulatorPositionExtended)
            )*/, this
        );
    }

    public void extendCoralManipulatorToPercentage(double percent) {
        m_desiredPosition = EncoderConstants.kDesiredManipulatorPositionExtended * percent;
    }

    /**Retracts the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to retract.
     */
    public Command retractCoralManipulator() {
        // Define a new command that moves the TalonFX to the desired retraction
        return Commands.runOnce(() -> 
            {m_desiredPosition = 0;} 
            /*m_manipulatorAngleMotor.setControl(
                m_motionMagicVoltage.withPosition(
                    EncoderConstants.kDesiredManipulatorPositionRetracted)
            )*/, this
        );
    }

    public void incrementManipulatorPosition() {
        m_positionPointer = Math.min(m_positionPointer + 1, 2);
        m_desiredPosition = EncoderConstants.kDesiredManipulatorPositionExtended * m_manipulatorIncrements[m_positionPointer];
    }

    public void decrementManipulatorPosition() {
        m_positionPointer = Math.max(m_positionPointer - 1, 0);
        m_desiredPosition = EncoderConstants.kDesiredManipulatorPositionExtended * m_manipulatorIncrements[m_positionPointer];
    }

    public boolean getEncoderPressed() {
        return m_manipulatorDriveMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    @Override
    public void periodic() {
        m_manipulatorAngleMotor.setControl(
            m_motionMagicVoltage.withPosition(
                m_desiredPosition)
        );
    }
}
