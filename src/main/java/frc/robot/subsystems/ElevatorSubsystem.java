package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.commands.ElevatorHomingCommand;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorDrive = new TalonFX(CANConstants.kElevatorDriveID);

    private final TalonFXConfiguration m_elevatorDriveConfiguration = new TalonFXConfiguration();

    private final DigitalInput m_opticalSensor = new DigitalInput(DIOConstants.kElevatorOpticalSensorID); 

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorSubsystem() {
        applyMotorConfigurations();

        setDefaultCommand(new ElevatorHomingCommand(this));
    }

    public void applyMotorConfigurations() {
        Slot0Configs elevatorSlot0 = m_elevatorDriveConfiguration.Slot0;

        MotionMagicConfigs elevatorMotionMagic = m_elevatorDriveConfiguration.MotionMagic;

        elevatorSlot0.kS = 0.24;
        elevatorSlot0.kV = 0.12;
        elevatorSlot0.kP = 0.11;
        elevatorSlot0.kI = 0.5;
        elevatorSlot0.kD = 0.001;

        elevatorMotionMagic.MotionMagicCruiseVelocity = 80; 
        elevatorMotionMagic.MotionMagicAcceleration   = 160;
        elevatorMotionMagic.MotionMagicJerk           = 1600;

        m_elevatorDrive.getConfigurator().apply(
            m_elevatorDriveConfiguration, 0.050);
        
        m_elevatorDrive.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean isSensorTriggered() {
        return m_opticalSensor.get();
    }

    public void moveElevator(double speed) {
        DutyCycleOut elevatorDriveSpeed = new DutyCycleOut(speed);

        m_elevatorDrive.setControl(elevatorDriveSpeed);
    }

    public void moveElevatorToPosition(double position) {
        m_elevatorDrive.setControl(m_motionMagicVoltage.withPosition(position));
    }

    public Command moveElevatorToPositionCommand(double position) {
        return Commands.runOnce(() -> moveElevatorToPosition(position), this);
    }

    public void resetEncoder() {
        m_elevatorDrive.setPosition(0);
    }

    @Override
    public void periodic() {}
}
