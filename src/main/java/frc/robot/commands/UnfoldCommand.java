package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class UnfoldCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final TiltRampSubsystem m_tiltRampSubsystem;

  public UnfoldCommand(
      ManipulatorSubsystem manipulatorSubsystem,
      TiltRampSubsystem tiltRampSubsystem,
      RobotContainer robotContainer) {
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_tiltRampSubsystem = tiltRampSubsystem;
  }

  @Override
  public void initialize() {
    m_tiltRampSubsystem.moveToPosition(-75);
  }

  @Override
  public boolean isFinished() {
    return m_tiltRampSubsystem.getErrorFromTarget() < 0.5
        && m_manipulatorSubsystem.getErrorFromTarget() < 0.5;
  }
}
