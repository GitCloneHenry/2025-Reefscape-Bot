package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncrementElevatorToLevelCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final DriveSubsystem m_driveSubsystem;

    private final int m_desiredElevatorLevel;
    
    private long m_startTime;

    public IncrementElevatorToLevelCommand(ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, int desiredElevatorLevel) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_desiredElevatorLevel = desiredElevatorLevel; 
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.incrementElevatorToLevel(m_desiredElevatorLevel);
        m_driveSubsystem.enableSlowMode();
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.getErrorFromTarget() < 0.5 && System.currentTimeMillis() - m_startTime > 100;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.disableSlowMode();
    }
}
