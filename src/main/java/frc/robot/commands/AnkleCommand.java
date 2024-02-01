// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class AnkleCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_subsystem;
  private final CommandXboxController m_controller;
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_IntakeSubsystem The subsystem used by this command.
   */
  public AnkleCommand(IntakeSubsystem m_IntakeSubsystem,CommandXboxController controller) {
    m_subsystem = m_IntakeSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getLeftX() > 0 ) {
      System.out.println("up");
      m_subsystem.ankleMotor.set(1);
    } else if (m_controller.getLeftX() < 0 ) {
      System.out.println("down");
      m_subsystem.ankleMotor.set(-1);
    } else {
      System.out.println("halt");
      m_subsystem.ankleMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.ankleMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_subsystem.toplimitSwitch.get() && m_controller.getLeftX() > 0) || (m_subsystem.bottomlimitSwitch.get() && m_controller.getLeftX() < 0)) {
      return true; // returns true when trying to move into a limit switch
    }
    return false;
  }
}
