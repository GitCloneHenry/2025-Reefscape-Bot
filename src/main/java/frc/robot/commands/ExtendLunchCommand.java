package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ExtendLunchCommand extends Command {
  private final BillsLunchSubsystem m_billsLunchSubsystem;
  private final ManipulatorSubsystem m_manipulatorSubsystem;

  public ExtendLunchCommand(
      BillsLunchSubsystem billsLunchSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    m_billsLunchSubsystem = billsLunchSubsystem;
    m_manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    m_manipulatorSubsystem.setSpeed(0.5);
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.05);
    m_billsLunchSubsystem.setPosition(-100);
  }

  @Override
  public boolean isFinished() {
    return m_manipulatorSubsystem.getEncoderPressed();
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulatorSubsystem.setSpeed(0.2);
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.5);
    m_billsLunchSubsystem.setPosition(0);
  }
}
