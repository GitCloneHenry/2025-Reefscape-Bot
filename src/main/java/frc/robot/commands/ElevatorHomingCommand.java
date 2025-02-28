package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHomingCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public ElevatorHomingCommand(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override 
    public void initialize() {
        m_elevatorSubsystem.moveElevator(-0.01);
    }

    @Override
    public void execute() {
        if (m_elevatorSubsystem.isSensorTriggered()) {
            m_elevatorSubsystem.moveElevator(0.0);
            m_elevatorSubsystem.resetEncoder();
        }
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isSensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.moveElevator(0.0);
    }
}
