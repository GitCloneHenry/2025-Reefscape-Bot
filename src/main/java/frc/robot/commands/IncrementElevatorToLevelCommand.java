package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncrementElevatorToLevelCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final int m_desiredElevatorLevel;
    
    private long m_startTime;

    public IncrementElevatorToLevelCommand(ElevatorSubsystem elevatorSubsystem, int desiredElevatorLevel) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_desiredElevatorLevel = desiredElevatorLevel; 
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.incrementElevatorToLevel(m_desiredElevatorLevel);
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.getErrorFromTarget() < 2.0 && System.currentTimeMillis() - m_startTime > 100;
    }
}
