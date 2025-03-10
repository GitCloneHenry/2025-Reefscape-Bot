package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class OuttakeCoralCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;

  public OuttakeCoralCommand(ManipulatorSubsystem manipulatorSubsystem) {
    m_manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    m_manipulatorSubsystem.setSpeed(-0.5);
  }

  @Override
  public boolean isFinished() {
    return m_manipulatorSubsystem.getEncoderPressed();
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulatorSubsystem.setSpeed(0.0);
  }
}
