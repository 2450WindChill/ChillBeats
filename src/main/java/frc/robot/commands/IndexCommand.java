// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IndexCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LauncherSubsystem m_subsystem;
  private final double m_speed;

  /**
   * Creates a new IndexCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexCommand(LauncherSubsystem subsystem, double speed) {
    m_subsystem = subsystem;

    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     System.out.println("INDEX INITIALIZE");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("INDEX EXECUTING");
    m_subsystem.feederMotor.set(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     System.out.println("INDEX END");
    m_subsystem.feederMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
  // GOAL FOR 1/31:: Make it stop at 4. !!
}
