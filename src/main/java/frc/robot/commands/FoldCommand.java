package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class FoldCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final ManipulatorSubsystem m_manipulatorSubsystem;
    private final TiltRampSubsystem m_tiltRampSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final RobotContainer m_robotContainer;

    public FoldCommand(ClimberSubsystem climberSubsystem, ManipulatorSubsystem manipulatorSubsystem, TiltRampSubsystem tiltRampSubsystem, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, RobotContainer robotContainer) {
        m_climberSubsystem = climberSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;
        m_tiltRampSubsystem = tiltRampSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setClimberPosition(0);
        m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.4);
        m_tiltRampSubsystem.moveToPosition(-75 * 20 / 90);
        m_elevatorSubsystem.moveElevatorToPosition(0.0);
        m_driveSubsystem.enableSlowMode();

        m_climberSubsystem.removeDefaultCommand();
    }

    @Override
    public boolean isFinished() {
        return m_tiltRampSubsystem.getErrorFromTarget() < 0.5 && m_manipulatorSubsystem.getErrorFromTarget() < 0.5 && m_climberSubsystem.getErrorFromTarget() < 0.5 && m_elevatorSubsystem.getErrorFromTarget() < 0.5;
    }

    @Override 
    public void end(boolean interrupted) {
        m_driveSubsystem.disableSlowMode();
        m_climberSubsystem.setDefaultCommand(m_robotContainer.m_defaultClimbCommand);
    }
}
