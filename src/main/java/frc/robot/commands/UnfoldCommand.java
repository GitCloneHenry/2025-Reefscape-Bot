package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EncoderConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class UnfoldCommand extends Command {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final TiltRampSubsystem m_tiltRampSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final RobotContainer m_robotContainer;

  public UnfoldCommand(
      ManipulatorSubsystem manipulatorSubsystem,
      TiltRampSubsystem tiltRampSubsystem,
      ClimberSubsystem climberSubsystem,
      RobotContainer robotContainer) {
    m_manipulatorSubsystem = manipulatorSubsystem;
    m_tiltRampSubsystem = tiltRampSubsystem;
    m_climberSubsystem = climberSubsystem;
    m_robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    m_tiltRampSubsystem.moveToPosition(-75 / 2);
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(1.10);
    m_climberSubsystem.setClimberPosition(EncoderConstants.kMaximumAcceptableClimberPosition);
    m_climberSubsystem.removeDefaultCommand();
  }

  @Override
  public boolean isFinished() {
    return m_tiltRampSubsystem.getErrorFromTarget() < 0.5
        && m_manipulatorSubsystem.getErrorFromTarget() < 0.5
        && m_climberSubsystem.getErrorFromTarget() < 0.5;
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setDefaultCommand(m_robotContainer.m_defaultClimbCommand);
  }
}
