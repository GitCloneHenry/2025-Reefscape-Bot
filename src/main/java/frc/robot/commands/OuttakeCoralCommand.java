package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class OuttakeCoralCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;

  private long m_startTime = 0;

  public OuttakeCoralCommand(ManipulatorSubsystem manipulatorSubsystem) {
    m_manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    Command currentManipulatorCommand= m_manipulatorSubsystem.getCurrentCommand();

    if (currentManipulatorCommand != null) {
      m_manipulatorSubsystem.getCurrentCommand().cancel();
    }
    
    m_manipulatorSubsystem.setSpeed(-0.10);
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - m_startTime > 1000;
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulatorSubsystem.setSpeed(0.0);
  }
}
