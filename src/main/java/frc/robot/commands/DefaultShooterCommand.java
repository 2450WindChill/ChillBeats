package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;

public class DefaultShooterCommand extends Command {
    private LauncherSubsystem m_launcherSubsystem;
    private LightySubsystem m_ledSubsytem;
    public DefaultShooterCommand(LauncherSubsystem launcherSubsystem) {
        m_launcherSubsystem = launcherSubsystem;
        addRequirements(m_launcherSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        m_launcherSubsystem.bottomMotor.set(-0.05);
        m_launcherSubsystem.topMotor.set(0.05);

        if (!m_launcherSubsystem.wristBeamBreak.get()) {
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
