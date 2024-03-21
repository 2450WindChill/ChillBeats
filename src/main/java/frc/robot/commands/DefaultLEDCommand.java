package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;

public class DefaultLEDCommand extends Command {

    private LauncherSubsystem m_LauncherSubsystem;
    private LightySubsystem m_LightySubsystem;
    public DefaultLEDCommand(LauncherSubsystem launcherSubsystem, LightySubsystem lightySubsystem) {
        m_LauncherSubsystem = launcherSubsystem;
        m_LightySubsystem = lightySubsystem;
        addRequirements(m_LightySubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        if (!m_LauncherSubsystem.wristBeamBreak.get()) {
            m_LightySubsystem.SetLEDsToGreen();
        } else {
            m_LightySubsystem.SetLEDsToAlliance();
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {

    }
}
