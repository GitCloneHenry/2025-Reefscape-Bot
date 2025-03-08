package frc.robot;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DeOrigamiCommand;
import frc.robot.commands.ElevatorHomingCommand;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = 
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

  private final ManipulatorSubsystem m_manipulatorSubsystem = 
      new ManipulatorSubsystem();

  private final BillsLunchSubsystem m_billsLunchSubsystem = 
      new BillsLunchSubsystem();

  private final TiltRampSubsystem m_tiltRampSubsystem =
      new TiltRampSubsystem();

  // private final FloorIntakeSubsystem m_floorIntakeSubsystem = 
  //     new FloorIntakeSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = 
      new ElevatorSubsystem();

  private final ClimberSubsystem m_climberSubsystem = 
      new ClimberSubsystem();

  private final DeOrigamiCommand m_deOrigamiCommand; 

  public RobotContainer() {
    m_deOrigamiCommand = new DeOrigamiCommand(null, m_manipulatorSubsystem, m_tiltRampSubsystem);
    waitMillis(5000);
    m_climberSubsystem.applyMotorConfigurations();
    // m_floorIntakeSubsystem.applyMotorConfigurations();
    m_manipulatorSubsystem.applyMotorConfigurations();
    m_tiltRampSubsystem.applyMotorConfigurations();
    configureBindings();
  }

  public void waitMillis(double milliseconds) {
    long startTime = System.currentTimeMillis();
    while (System.currentTimeMillis() - startTime < milliseconds) {}
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

    

    // copilotControllerA.onTrue(Commands.runOnce(
    //   () -> {
    //     m_tiltRampSubsystem.moveToPossition(-75 * 11 / 15);
    //     m_tiltRampSubsystem.setDrivePower(1500);
    //   }, m_tiltRampSubsystem
    // ));
    // copilotControllerB.onTrue(Commands.runOnce(
    //   () -> {
    //     m_tiltRampSubsystem.moveToPossition(0);
    //     m_tiltRampSubsystem.setDrivePower(0);
    //   }, m_tiltRampSubsystem
    // ));
    // copilotControllerY.onTrue(Commands.runOnce(
    //   () -> {
    //     m_elevatorSubsystem.incrementElevatorPosition();
    //     m_manipulatorSubsystem.incrementManipulatorPosition();
    //   }
    // ));
    // copilotControllerX.onTrue(Commands.runOnce(
    //   () -> {
    //     m_elevatorSubsystem.decrementElevatorPosition();
    //     m_manipulatorSubsystem.decrementManipulatorPosition();
    //   }
    // ));

    // copilotControllerLS.onTrue(Commands.run(
    //   () -> {
    //     System.err.println("uwoopsie, looks liek someone forgor to make this~ :3 :L");
    //   }
    // ));

    // copilotControllerRS.onTrue(Commands.run(
    //   () -> {
    //     System.err.println("uwoopsie, looks liek someone forgor to make this~ :3 :L");
    //   }
    // ));

    // copilotControllerLT.onTrue(Commands.run(
    //   () -> {
    //     m_manipulatorSubsystem.setSpeed(0.0);
    //   }
    // ));

    // copilotControllerRT.onTrue(Commands.run(
    //   () -> {
    //     m_manipulatorSubsystem.setSpeed(-0.5);
    //   }
    // ));

    // m_climberSubsystem.setDefaultCommand(
    //   new RunCommand(() -> m_climberSubsystem.incrementClimberPosition(m_copilotController.getLeftY()), m_climberSubsystem));







    // driverControllerX.onTrue(Commands.run(() -> {
    //   m_manipulatorSubsystem.setSpeed(0.5);
    //   m_billsLunchSubsystem.setPosition(-100);
    // }));
    // driverControllerX.onFalse(Commands.run(() -> {
    //   m_manipulatorSubsystem.setSpeed(0.1);
    //   m_billsLunchSubsystem.setPosition(0);
    // }));
    // driverControllerU.onTrue(Commands.run(() -> m_manipulatorSubsystem.extendCoralManipulatorToPercentage(122.5 / 90.0), m_manipulatorSubsystem));
    // driverControllerU.onTrue(Commands.run(() -> m_manipulatorSubsystem.extendCoralManipulatorToPercentage(1.85), m_manipulatorSubsystem));
    // driverControllerD.onTrue(m_manipulatorSubsystem.retractCoralManipulator());
    // driverControllerL.onTrue(m_tiltRampSubsystem.moveToPossition(-75 * 55 / 75));
    // driverControllerR.onTrue(m_tiltRampSubsystem.moveToPossition(0));
    // driverControllerLC.onTrue(m_tiltRampSubsystem.setDrivePower(1500));
    // driverControllerRC.onTrue(m_tiltRampSubsystem.setDrivePower(-1500));

    // m_elevatorSubsystem.setDefaultCommand(
    //   new RunCommand(() -> m_elevatorSubsystem.applyElevatorSpeed(-m_driverController.getRightY() / 20), m_elevatorSubsystem));

    // driverControllerA.onTrue(Commands.run(() -> {
    //   m_elevatorSubsystem.moveElevatorToPosition(0.0);
    //   m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.5);
    // }, m_elevatorSubsystem, m_manipulatorSubsystem));
    // driverControllerB.onTrue(Commands.run(() -> {
    //   m_elevatorSubsystem.moveElevatorToPosition(-420 * 36 / 100);
    //   m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.5);
    // }, m_elevatorSubsystem, m_manipulatorSubsystem));
    // driverControllerA.onTrue(m_elevatorSubsystem.moveElevatorToPositionCommand(0.0));
    // driverControllerB.onTrue(m_elevatorSubsystem.moveElevatorToPositionCommand(-420.0));
  }

  public void startHomingProcess() {
    ElevatorHomingCommand elevatorHomingCommand = new ElevatorHomingCommand(m_elevatorSubsystem); 

    elevatorHomingCommand.andThen(m_deOrigamiCommand).schedule();
  }
}
