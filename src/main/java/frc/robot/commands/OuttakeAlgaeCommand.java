package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TiltRampSubsystem;

public class OuttakeAlgaeCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;
  private final RobotContainer m_robotContainer;

  private long m_startTime;

  public OuttakeAlgaeCommand(TiltRampSubsystem tiltRampSubsystem, RobotContainer robotContainer) {
    m_tiltRampSubsystem = tiltRampSubsystem;
    m_robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    m_robotContainer.m_intakeAlgaeCommand.cancel();

    m_tiltRampSubsystem.moveToPosition(-75 * 20 / 90);
    m_tiltRampSubsystem.setDrivePower(1500);

    m_startTime = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return !m_tiltRampSubsystem.getMotorStalled() && System.currentTimeMillis() - m_startTime > 1000;
  }

  @Override
  public void end(boolean interrupted) {
    m_tiltRampSubsystem.setDrivePower(0);
  }
}
