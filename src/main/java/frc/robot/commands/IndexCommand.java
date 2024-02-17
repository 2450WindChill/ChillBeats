// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IndexCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IndexSubsystem m_subsystem;
private final CommandXboxController m_controller;
  /**
   * Creates a new IndexCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexCommand(IndexSubsystem subsystem,CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getLeftX() > 0 ) {
      m_subsystem.indexMotor.set(1);
    }
    else if (m_controller.getLeftX() < 0 ) {
      m_subsystem.indexMotor.set(-1);
    }
    else {
      m_subsystem.indexMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.indexMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
   
  }
// GOAL FOR 1/31:: Make it stop at 4. !!
}
