package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltRampSubsystem;

public class IntakeAlgaeCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;

  public IntakeAlgaeCommand(TiltRampSubsystem tiltRampSubsystem) {
    m_tiltRampSubsystem = tiltRampSubsystem;
  }

  @Override
  public void initialize() {
    m_tiltRampSubsystem.moveToPossition(-75 * 55 / 90);
    // m_tiltRampSubsystem.set
  }
}
