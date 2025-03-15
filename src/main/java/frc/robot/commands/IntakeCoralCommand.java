package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

public class IntakeCoralCommand extends Command {
  private final TiltRampSubsystem m_tiltRampSubsystem;
  private final ManipulatorSubsystem m_manipulatorSubsystem;

  private long m_startTime = 0;

  public IntakeCoralCommand(TiltRampSubsystem tiltRampSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    m_tiltRampSubsystem = tiltRampSubsystem;
    m_manipulatorSubsystem = manipulatorSubsystem;
  }

  @Override
  public void initialize() {
    m_manipulatorSubsystem.setSpeed(0.5);
    m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.00);
    m_tiltRampSubsystem.moveToPosition(-75 * 30 / 90);

    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // if (m_tiltRampSubsystem.getErrorFromTarget() < 0.3 && m_manipulatorSubsystem.getErrorFromTarget() < 0.3 && System.currentTimeMillis() - m_startTime > 1000) {
    //   System.err.println(m_tiltRampSubsystem.getErrorFromTarget() + "," + m_manipulatorSubsystem.getErrorFromTarget());
    // }
  }

  @Override
  public boolean isFinished() {
    return m_tiltRampSubsystem.getErrorFromTarget() < 0.3 && m_manipulatorSubsystem.getErrorFromTarget() < 0.3 && System.currentTimeMillis() - m_startTime > 1000;
    // return false;
  }
}
