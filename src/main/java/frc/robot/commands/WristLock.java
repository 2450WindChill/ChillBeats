// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristLock extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final AimSubsystem m_subsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private double rotationTarget;
  private boolean isGoingUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristLock(AimSubsystem subsystem, LauncherSubsystem launcherSubsystem, Double rotationTarget) {
    m_subsystem = subsystem;
    m_launcherSubsystem = launcherSubsystem;
    this.rotationTarget = rotationTarget;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, m_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = m_subsystem.wristMotor.getEncoder().getPosition();

    if (currentPosition < rotationTarget) {
      isGoingUp = true;
    } else {
      isGoingUp = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.wristController.setReference(rotationTarget, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending move to Wrist");
    m_subsystem.wristMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_launcherSubsystem.wristBeamBreak.get();
  }
}
