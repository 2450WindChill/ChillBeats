// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SourceIntakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IndexSubsystem m_indexSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private boolean currentBeamBreakState;
  private int stateChangeCounter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SourceIntakeCommand(IndexSubsystem indexSubsystem, LauncherSubsystem launcherSubsystem) {
    m_indexSubsystem = indexSubsystem;
    m_launcherSubsystem = launcherSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexSubsystem, launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexSubsystem.indexMotor.set(0.2);
    m_launcherSubsystem.topMotor.set(-0.4);
    m_launcherSubsystem.bottomMotor.set(0.4);
    System.err.println("Full Intake Init");
    currentBeamBreakState = m_indexSubsystem.beamBreak.get();
    stateChangeCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.err.println(m_indexSubsystem.beamBreak.get());
    if (currentBeamBreakState != m_indexSubsystem.beamBreak.get()) {
      stateChangeCounter += 1;
      currentBeamBreakState = m_indexSubsystem.beamBreak.get();
      System.err.println("Beam Break Changed");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexSubsystem.indexMotor.set(0);
    m_launcherSubsystem.topMotor.set(0);
    m_launcherSubsystem.bottomMotor.set(0);
    System.err.println("Full Intake End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stateChangeCounter >= 2) {
      return true;
    } else {
      return false;
    }
  }
}
