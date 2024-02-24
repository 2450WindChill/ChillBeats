package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class DefaultShooterCommand extends Command {
    private LauncherSubsystem m_launcherSubsystem;
    public DefaultShooterCommand(LauncherSubsystem launcherSubsystem) {
        m_launcherSubsystem = launcherSubsystem;
        addRequirements(m_launcherSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        m_launcherSubsystem.bottomMotor.set(-0.05);
        m_launcherSubsystem.topMotor.set(0.05);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {

    }
}
