package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltRampSubsystem;

public class IntakeCoralCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;

  public IntakeCoralCommand(TiltRampSubsystem tiltRampSubsystem) {
    m_tiltRampSubsystem = tiltRampSubsystem;
  }

  @Override
  public void initialize() {
    m_tiltRampSubsystem.moveToPossition(-75 * 40 / 90);
  }

  @Override
  public boolean isFinished() {
    return m_tiltRampSubsystem.getErrorFromTarget() < 0.5;
  }
}
