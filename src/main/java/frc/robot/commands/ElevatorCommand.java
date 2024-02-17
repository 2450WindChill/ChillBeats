// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ElevatorSubsystem m_subsystem;
  private final CommandXboxController m_controller;
  
  /**
   * Creates a new ElevatorCommand.
   *
   * @param m_ElevatorSubsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem m_ElevatorSubsystem,CommandXboxController controller) {
    m_subsystem = m_ElevatorSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getLeftX() > 0 ) {
      m_subsystem.elevatorMotor.set(0.1);
    }
    else if (m_controller.getLeftX() < 0 ) {
      m_subsystem.elevatorMotor.set(-0.1);
    }
    else {
      m_subsystem.elevatorMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.elevatorMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
//     boolean descent = m_controller.getLeftX() < 0;
//     boolean ascent = m_controller.getLeftX() > 0;
// //descent and ascent to control when the control stops going up and down== no crashing into ground or robot. 
//     if ((m_subsystem.elevatorMotor.getEncoder().getPosition() > 4) && ascent) return true;
//     else if ((m_subsystem.elevatorMotor.getEncoder().getPosition() < 0) && descent) return true;
//     else return false;
  }
}
