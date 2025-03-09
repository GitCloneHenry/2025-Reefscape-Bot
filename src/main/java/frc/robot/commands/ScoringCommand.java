package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BillsLunchSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TiltRampSubsystem;

// driverControllerX.onTrue(new RunCommand(() -> {
//     m_manipulatorSubsystem.setSpeed(0.5);
//     m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.07);
//     m_billsLunchSubsystem.setPosition(-100);
//     m_tiltRampSubsystem.moveToPossition(-75 * 40 / 90);
//   }, m_manipulatorSubsystem, m_billsLunchSubsystem, m_tiltRampSubsystem));

public class ScoringCommand extends Command {
    private final ManipulatorSubsystem m_manipulatorSubsystem; 
    private final BillsLunchSubsystem m_billsLunchSubsystem;
    private final TiltRampSubsystem m_tiltRampSubsystem;

    public ScoringCommand(ManipulatorSubsystem manipulatorSubsystem, BillsLunchSubsystem billsLunchSubsystem, TiltRampSubsystem tiltRampSubsystem) {
        m_manipulatorSubsystem = manipulatorSubsystem;
        m_billsLunchSubsystem = billsLunchSubsystem;
        m_tiltRampSubsystem = tiltRampSubsystem;
    }

    @Override 
    public void initialize() {
        m_tiltRampSubsystem.moveToPossition(-75 * 40 / 90);
    }

    @Override
    public void execute() {
        if (Math.abs(m_tiltRampSubsystem.getPosition() - (-75 * 40 / 90)) <= 0.1) {
            m_manipulatorSubsystem.setSpeed(0.5);
            m_manipulatorSubsystem.extendCoralManipulatorToPercentage(0.05);
            m_billsLunchSubsystem.setPosition(-110);
        }
    }

    @Override
    public boolean isFinished() {
        return m_manipulatorSubsystem.getEncoderPressed();
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulatorSubsystem.setSpeed(0.2);
        m_billsLunchSubsystem.setPosition(0);
    }
}
