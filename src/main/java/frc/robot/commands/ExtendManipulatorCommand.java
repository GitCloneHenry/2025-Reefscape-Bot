package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ExtendManipulatorCommand extends Command {
    private final ManipulatorSubsystem m_manipulatorSubsystem;

    private double m_target;
    
    public ExtendManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, double target) {
        m_manipulatorSubsystem = manipulatorSubsystem;
        m_target = target;
    }

    @Override
    public void initialize() {
        m_manipulatorSubsystem.extendCoralManipulatorToPercentage(m_target);
    }

    @Override
    public boolean isFinished() {
        return m_manipulatorSubsystem.getErrorFromTarget() < 1.0;
    }
}
