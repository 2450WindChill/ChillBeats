package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LightySubsystem;

public class DefaultIndexCommand extends Command{
    private IndexSubsystem m_indexSubsystem;
    private LightySubsystem m_ledSubsytem;

    public DefaultIndexCommand(IndexSubsystem indexSubsystem, LightySubsystem ledSubsystem) {
        m_indexSubsystem = indexSubsystem;
        m_ledSubsytem = ledSubsystem;

        addRequirements(indexSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        if (!m_indexSubsystem.beamBreak.get()) {
            m_ledSubsytem.SetLEDsToGreen();
        } else {
            m_ledSubsytem.SetLEDsToAlliance();
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {

    }
}
