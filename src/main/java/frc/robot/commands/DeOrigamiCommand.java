package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class DeOrigamiCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final TiltRampSubsystem m_tiltRampSubsystem;

  Command tiltRampCommand;
  Command manipulatorCommand;

  public DeOrigamiCommand(
      ManipulatorSubsystem manipulatorSubsystem, TiltRampSubsystem tiltRampSubsystem) {
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_tiltRampSubsystem = tiltRampSubsystem;

    tiltRampCommand = m_tiltRampSubsystem.moveToPossitionCommand(-75 / 2);
    // Command floorIntakeCommand = m_floorIntakeSubsystem
    manipulatorCommand = m_manipulatorSubsystem.extendCoralManipulator();
  }

  @Override
  public void initialize() {
    tiltRampCommand.andThen(manipulatorCommand).schedule();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return tiltRampCommand.isFinished() && manipulatorCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {}
}
