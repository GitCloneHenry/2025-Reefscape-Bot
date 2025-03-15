package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorHomingCommand;
import frc.robot.commands.ExtendElevatorCommand;
import frc.robot.commands.ExtendLunchCommand;
import frc.robot.commands.FoldCommand;
import frc.robot.commands.IntakeAlgaeCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.OuttakeAlgaeCommand;
import frc.robot.commands.OuttakeCoralCommand;
import frc.robot.commands.TagCenteringCommand;
import frc.robot.commands.UnfoldCommand;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;
import org.photonvision.PhotonCamera;

public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The camera used for april tag detection
  public final PhotonCamera m_aprilTagCamera = new PhotonCamera("USB_ATag_Camera");

  public final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public final CommandXboxController m_copilotController =
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();

  private final BillsLunchSubsystem m_billsLunchSubsystem = new BillsLunchSubsystem();

  private final TiltRampSubsystem m_tiltRampSubsystem = new TiltRampSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  public final RunCommand m_defaultDriveCommand;
  public final RunCommand m_defaultClimbCommand;
  public final RunCommand m_defaultManipulatorCommand;

  public RobotContainer() {
    m_defaultDriveCommand =
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY() * 0.7, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX() * 0.7, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX() * 0.7, OIConstants.kDriveDeadband),
                    true),
            m_robotDrive);
    m_defaultClimbCommand =
        new RunCommand(
            () -> m_climberSubsystem.incrementClimberPosition(m_copilotController.getRightY()),
            m_climberSubsystem);
    m_defaultManipulatorCommand =
        new RunCommand(
            () -> m_manipulatorSubsystem.applyElevatorOffset(-m_copilotController.getLeftY() * 0.5),
            m_manipulatorSubsystem);

    applyMotorConfigurations();
    configureBindings();
  }

  public void waitMillis(double milliseconds) {
    long startTime = System.currentTimeMillis();
    while (System.currentTimeMillis() - startTime < milliseconds) {}
  }

  public void applyMotorConfigurations() {
    waitMillis(5000);
    m_climberSubsystem.applyMotorConfigurations();
    m_manipulatorSubsystem.applyMotorConfigurations();
    m_tiltRampSubsystem.applyMotorConfigurations();
  }

  private void configureCommands() {}

  private void configureBindings() {
    Trigger driverControllerA = m_driverController.a(); // Driver's A Button
    Trigger driverControllerB = m_driverController.b(); // Driver's B Button
    Trigger driverControllerX = m_driverController.x(); // Driver's X Button
    Trigger driverControllerY = m_driverController.y(); // Driver's Y Button

    Trigger driverControllerU = m_driverController.povUp(); // Driver's DPAD Up
    Trigger driverControllerD = m_driverController.povDown(); // Driver's DPAD Down
    Trigger driverControllerL = m_driverController.povLeft(); // Driver's DPAD Left
    Trigger driverControllerR = m_driverController.povRight(); // Driver's DPAD Right

    Trigger driverControllerLB = m_driverController.leftBumper(); // Driver's Left Bumper
    Trigger driverControllerRB = m_driverController.rightBumper(); // Driver's Right Bumper
    Trigger driverControllerLT = m_driverController.leftTrigger(); // Driver's Left Trigger
    Trigger driverControllerRT = m_driverController.rightTrigger(); // Driver's Right Trigger

    Trigger driverControllerLC = m_driverController.leftStick(); // Driver's Left Stick (Click)
    Trigger driverControllerRC = m_driverController.rightStick(); // Driver's Right Stick (Click)

    Trigger copilotControllerA = m_copilotController.a(); // Copilot's A Button
    Trigger copilotControllerB = m_copilotController.b(); // Copilot's B Button
    Trigger copilotControllerX = m_copilotController.x(); // Copilot's X Button
    Trigger copilotControllerY = m_copilotController.y(); // Copilot's Y Button

    Trigger copilotControllerU = m_copilotController.povUp(); // Copilot's DPAD Up
    Trigger copilotControllerD = m_copilotController.povDown(); // Copilot's DPAD Down
    Trigger copilotControllerL = m_copilotController.povLeft(); // Copilot's DPAD Left
    Trigger copilotControllerR = m_copilotController.povRight(); // Copilot's DPAD Right

    Trigger copilotControllerLB = m_copilotController.leftBumper(); // Copilot's Left Bumper
    Trigger copilotControllerRB = m_copilotController.rightBumper(); // Copilot's Right Bumper
    Trigger copilotControllerLT = m_copilotController.leftTrigger(); // Copilot's Left Trigger
    Trigger copilotControllerRT = m_copilotController.rightTrigger(); // Copilot's Right Trigger

    Trigger copilotControllerLC = m_copilotController.leftStick(); // Copilot's Left Stick (Click)
    Trigger copilotControllerRC = m_copilotController.rightStick(); // Copilot's Right Stick (Click)

    driverControllerA.onTrue(new IntakeAlgaeCommand(m_tiltRampSubsystem));
    driverControllerB.onTrue(new OuttakeAlgaeCommand(m_tiltRampSubsystem));

    driverControllerLT.onTrue(m_robotDrive.enableSlowModeCommand());
    driverControllerLT.onFalse(m_robotDrive.disableSlowModeCommand());

    driverControllerLB.onTrue(m_robotDrive.zeroHeading());

    driverControllerL.onTrue(new TagCenteringCommand(this, new Translation2d(-0.15, 0.0)));
    driverControllerR.onTrue(new TagCenteringCommand(this, new Translation2d(0.15, 0.0)));

    copilotControllerA.whileTrue(m_tiltRampSubsystem.moveToPositionCommand(-75 * 35 / 90));
    copilotControllerA.onFalse(
        new IntakeCoralCommand(m_tiltRampSubsystem, m_manipulatorSubsystem)
            .andThen(new ExtendLunchCommand(m_billsLunchSubsystem, m_manipulatorSubsystem, m_elevatorSubsystem))
                .andThen(new ExtendElevatorCommand(m_elevatorSubsystem, m_manipulatorSubsystem))
                    .andThen(m_manipulatorSubsystem.extendCoralManipulator()));
    copilotControllerB.onTrue(new OuttakeCoralCommand(m_manipulatorSubsystem));
    copilotControllerX.onTrue(
        Commands.runOnce(
            () -> {
              m_elevatorSubsystem.decrementElevatorPosition();
              m_manipulatorSubsystem.decrementManipulatorPosition();
            },
            m_elevatorSubsystem,
            m_manipulatorSubsystem));
    copilotControllerY.onTrue(
        Commands.runOnce(
            () -> {
              m_elevatorSubsystem.incrementElevatorPosition();
              m_manipulatorSubsystem.incrementManipulatorPosition();
            },
            m_elevatorSubsystem,
            m_manipulatorSubsystem));

    copilotControllerLT.onTrue(
        new FoldCommand(
            m_climberSubsystem,
            m_manipulatorSubsystem,
            m_tiltRampSubsystem,
            m_elevatorSubsystem,
            m_robotDrive,
            this));
    copilotControllerRT.onTrue(
        new UnfoldCommand(m_manipulatorSubsystem, m_tiltRampSubsystem, m_climberSubsystem, this));
    
    copilotControllerU.onTrue(
      Commands.run(
        () -> {
          Command currentManipulatorCommand = m_manipulatorSubsystem.getCurrentCommand();

          if (currentManipulatorCommand != null) {
            currentManipulatorCommand.cancel();
          }

          m_manipulatorSubsystem.setDefaultCommand(m_defaultManipulatorCommand);
        },
        m_manipulatorSubsystem
      )
    );

    copilotControllerLC.onTrue(Commands.run(
      () -> m_manipulatorSubsystem.setSpeed(0.5), m_manipulatorSubsystem));
    copilotControllerRC.onTrue(Commands.run(
        () -> m_manipulatorSubsystem.extendCoralManipulatorToPercentage(1.0), m_manipulatorSubsystem));

    m_robotDrive.setDefaultCommand(m_defaultDriveCommand);
    m_climberSubsystem.setDefaultCommand(m_defaultClimbCommand);
  }

  public void runStartCommands() {
    ElevatorHomingCommand elevatorHomingCommand = new ElevatorHomingCommand(m_elevatorSubsystem);

    elevatorHomingCommand
        .andThen(
            new UnfoldCommand(
                m_manipulatorSubsystem, m_tiltRampSubsystem, m_climberSubsystem, this))
        .schedule();
  }
}
