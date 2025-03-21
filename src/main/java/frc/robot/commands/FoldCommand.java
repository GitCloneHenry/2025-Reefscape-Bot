package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class FoldCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final TiltRampSubsystem m_tiltRampSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final BillsLunchSubsystem m_billsLunchSubsystem;

  public FoldCommand(
      ManipulatorSubsystem manipulatorSubsystem,
      TiltRampSubsystem tiltRampSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      DriveSubsystem driveSubsystem,
      RobotContainer robotContainer,
      BillsLunchSubsystem billsLunchSubsystem) {
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_tiltRampSubsystem = tiltRampSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_billsLunchSubsystem = billsLunchSubsystem;
  }

  @Override
  public void initialize() {
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.1);
    m_tiltRampSubsystem.moveToPosition(-75 * 10 / 90);
    m_elevatorSubsystem.moveElevatorToPosition(0.0);
    m_driveSubsystem.enableSlowMode();
    m_billsLunchSubsystem.setPosition(0);

    m_elevatorSubsystem.resetIncrements();
  }

  @Override
  public boolean isFinished() {
    return m_tiltRampSubsystem.getErrorFromTarget() < 0.5
        && m_manipulatorSubsystem.getErrorFromTarget() < 0.5
        && m_elevatorSubsystem.getErrorFromTarget() < 0.5;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.disableSlowMode();
  }
}
