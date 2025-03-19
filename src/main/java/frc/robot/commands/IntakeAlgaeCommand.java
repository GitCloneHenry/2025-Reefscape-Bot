package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TiltRampSubsystem;

public class IntakeAlgaeCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;
  private final RobotContainer m_robotContainer;

  private long m_startStall = 0;
  private boolean m_motorStalled = false;

  public IntakeAlgaeCommand(TiltRampSubsystem tiltRampSubsystem, RobotContainer robotContainer) {
    m_tiltRampSubsystem = tiltRampSubsystem;
    m_robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    m_robotContainer.m_outtakeAlgaeCommand.cancel();
    
    m_tiltRampSubsystem.moveToPosition(-75 * 55 / 90);
    m_tiltRampSubsystem.setDrivePower(-1500);

    m_motorStalled = false;
    m_startStall = 0;
  }

  @Override
  public void execute() {
    if (m_tiltRampSubsystem.getMotorStalled() && !m_motorStalled) {
        m_motorStalled = true;
        m_startStall = System.currentTimeMillis();
    }
  }

  @Override
  public boolean isFinished() {
    return m_motorStalled && System.currentTimeMillis() - m_startStall > 1000;
  }

  @Override
  public void end(boolean interrupted) {
    m_tiltRampSubsystem.setDrivePower(-1000);
    m_tiltRampSubsystem.moveToPosition(-75 * 25 / 90);
  }
}
