package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    byte[] data = { DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? (byte) 4 : (byte) 5 }; 

    m_robotContainer.sendByteToStrip(data, 1);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    byte[] sendByteToStrip = { 1 };
    m_robotContainer.sendByteToStrip(sendByteToStrip, 1);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_robotContainer.runStartCommands().andThen(m_autonomousCommand).schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
