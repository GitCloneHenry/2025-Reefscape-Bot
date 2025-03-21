package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ExtendElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public long m_startTime = 0;

    public ExtendElevatorCommand(ElevatorSubsystem elevatorSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.moveElevatorToPosition(-10);

        m_startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.getErrorFromTarget() < 0.5 && System.currentTimeMillis() - m_startTime > 1000;
    }
}
