package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ExtendLunchCommand extends Command {
  private final BillsLunchSubsystem m_billsLunchSubsystem;
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;

  public ExtendLunchCommand(
      BillsLunchSubsystem billsLunchSubsystem, ManipulatorSubsystem manipulatorSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_billsLunchSubsystem = billsLunchSubsystem;
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void initialize() {
    m_manipulatorSubsystem.setSpeed(0.5);
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(-0.05);
    m_billsLunchSubsystem.setPosition(-110);
    // m_elevatorSubsystem.moveElevatorToPosition(-10);
  }

  @Override
  public void execute() {
    System.err.println(m_elevatorSubsystem.getErrorFromTarget());
  }

  @Override
  public boolean isFinished() {
    return m_manipulatorSubsystem.getEncoderPressed();
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulatorSubsystem.setSpeed(0.2);
    // m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.5);
    m_billsLunchSubsystem.setPosition(0);
  }
}
