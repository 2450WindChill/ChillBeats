// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveIntakeToPosCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_subsystem;
  private double rotationTarget;
  private boolean isDeploying;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveIntakeToPosCommand(IntakeSubsystem subsystem, Double rotationTarget) {
    m_subsystem = subsystem;
    this.rotationTarget = rotationTarget;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = m_subsystem.angleIntakeMotor.getEncoder().getPosition();
     System.out.println("Move intake initialize");

    if (currentPosition < rotationTarget) {
      isDeploying = true;
    } else {
      isDeploying = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_subsystem.angleIntakeMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Current intake position:", currentPosition);
    SmartDashboard.putNumber("Target intake position:", rotationTarget);
    SmartDashboard.putBoolean("Intake currently deploying?", isDeploying);
    System.out.println("Move intake execute " + currentPosition + " target angle: " + rotationTarget);


    m_subsystem.angleIntakeController.setReference(rotationTarget, ControlType.kPosition);
   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending intake deployment " + interrupted);
    m_subsystem.angleIntakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = m_subsystem.angleIntakeMotor.getEncoder().getPosition();
    if (isDeploying) {
      if (currentPosition >= rotationTarget - 0.05) {
        return true;
      } else {
        return false;
      }
    } else {

      if (currentPosition <= rotationTarget + 0.05) {
        return true;
      } else {
        return false;
      }
    }

  }
}
