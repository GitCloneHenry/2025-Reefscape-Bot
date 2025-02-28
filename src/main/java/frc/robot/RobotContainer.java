package frc.robot;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = 
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

  private final ManipulatorSubsystem m_manipulatorSubsystem;

  private final BillsLunchSubsystem m_billsLunchSubsystem = 
      new BillsLunchSubsystem();

  // private final ElevatorSubsystem m_elevatorSubsystem = 
  //     new ElevatorSubsystem();
  private final ClimberSubsystem m_climberSubsystem = 
      new ClimberSubsystem();

  public RobotContainer() {
    m_manipulatorSubsystem = new ManipulatorSubsystem();

    configureBindings();
  }

  private void configureBindings() {
    Trigger driverControllerA = m_driverController.a(); // Driver's A Button
    Trigger driverControllerB = m_driverController.b(); // Driver's B Button 
    Trigger driverControllerX = m_driverController.x(); // Driver's X Button
    Trigger driverControllerY = m_driverController.y(); // Driver's Y Button

    Trigger driverControllerU = m_driverController.povUp();    // Driver's DPAD Up
    Trigger driverControllerD = m_driverController.povDown();  // Driver's DPAD Down
    Trigger driverControllerL = m_driverController.povLeft();  // Driver's DPAD Left
    Trigger driverControllerR = m_driverController.povRight(); // Driver's DPAD Right

    Trigger driverControllerLS = m_driverController.leftBumper();   // Driver's Left Bumper
    Trigger driverControllerRS = m_driverController.rightBumper();  // Driver's Right Bumper
    Trigger driverControllerLT = m_driverController.leftTrigger();  // Driver's Left Trigger
    Trigger driverControllerRT = m_driverController.rightTrigger(); // Driver's Right Trigger 

    Trigger driverControllerLC = m_driverController.leftStick();  // Driver's Left Stick (Click)
    Trigger driverControllerRC = m_driverController.rightStick(); // Driver's Right Stick (Click)

    Trigger copilotControllerA = m_copilotController.a(); // Copilot's A Button
    Trigger copilotControllerB = m_copilotController.b(); // Copilot's B Button 
    Trigger copilotControllerX = m_copilotController.x(); // Copilot's X Button 
    Trigger copilotControllerY = m_copilotController.y(); // Copilot's Y Button 

    Trigger copilotControllerU = m_copilotController.povUp();    // Copilot's DPAD Up
    Trigger copilotControllerD = m_copilotController.povDown();  // Copilot's DPAD Down 
    Trigger copilotControllerL = m_copilotController.povLeft();  // Copilot's DPAD Left
    Trigger copilotControllerR = m_copilotController.povRight(); // Copilot's DPAD Right

    Trigger copilotControllerLS = m_copilotController.leftBumper();   // Copilot's Left Bumper
    Trigger copilotControllerRS = m_copilotController.rightBumper();  // Copilot's Right Bumper 
    Trigger copilotControllerLT = m_copilotController.leftTrigger();  // Copilot's Left Trigger
    Trigger copilotControllerRT = m_copilotController.rightTrigger(); // Copilot's Right Trigger

    Trigger copilotControllerLC = m_copilotController.leftStick();  // Copilot's Left Stick (Click)
    Trigger copilotControllerRC = m_copilotController.rightStick(); // Copilot's Right Stick (Click)

    driverControllerA.whileTrue(m_manipulatorSubsystem.getManipulatorDriveCommand(0));
    driverControllerB.whileTrue(m_manipulatorSubsystem.getManipulatorDriveCommand(-0.1));
    driverControllerX.onTrue(Commands.runOnce(() -> {
      m_manipulatorSubsystem.setSpeed(0.5);
      m_billsLunchSubsystem.setPosition(-100);
    }));
    driverControllerX.onFalse(Commands.runOnce(() -> {
      m_manipulatorSubsystem.setSpeed(0.1);
      m_billsLunchSubsystem.setPosition(0);
    }));
    driverControllerU.onTrue(m_manipulatorSubsystem.extendCoralManipulator());
    driverControllerD.onTrue(m_manipulatorSubsystem.retractCoralManipulator());

    m_climberSubsystem.setDefaultCommand(
      new RunCommand(() -> m_climberSubsystem.incrementClimberPosition(m_driverController.getLeftY()), m_climberSubsystem));
  }
}
