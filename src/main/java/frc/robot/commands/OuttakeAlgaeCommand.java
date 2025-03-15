package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltRampSubsystem;

public class OuttakeAlgaeCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;

  public OuttakeAlgaeCommand(TiltRampSubsystem tiltRampSubsystem) {
    m_tiltRampSubsystem = tiltRampSubsystem;
  }

  @Override
  public void initialize() {
    m_tiltRampSubsystem.moveToPosition(-75 * 20 / 90);
    m_tiltRampSubsystem.setDrivePower(1500);
  }

  @Override
  public boolean isFinished() {
    return !m_tiltRampSubsystem.getMotorStalled();
  }

  @Override
  public void end(boolean interrupted) {
    m_tiltRampSubsystem.setDrivePower(0);
  }
}
