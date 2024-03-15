package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;
// 
public class IntakeCommand extends Command{
    private IndexSubsystem m_indexSubsystem;
    private AimSubsystem m_aimSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;

    public IntakeCommand(IndexSubsystem indexSubsystem, AimSubsystem aimSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_indexSubsystem = indexSubsystem;
        m_aimSubsystem = aimSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;

        addRequirements(indexSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        // Motor moves unless the note is in the indexer but thge arm is not zeroed
        if (m_indexSubsystem.indexBeamBreak.get() && !isZero()) {
            m_indexSubsystem.indexMotor.set(0);
        } else {
            m_indexSubsystem.indexMotor.set(0.2);
        }
    }

    public boolean isZero() {
        return (m_aimSubsystem.wristMotor.getEncoder().getPosition() <= Constants.zeroLaunchAngle) 
                && (m_elevatorSubsystem.elevatorMotor.getEncoder().getPosition() <= Constants.zeroElevator);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {

    }
}
