// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveWristToPosCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final AimSubsystem m_subsystem;
  private double rotationTarget;
  private boolean isGoingUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveWristToPosCommand(AimSubsystem subsystem, Double rotationTarget) {
    m_subsystem = subsystem;
    this.rotationTarget = rotationTarget;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initializing move Wrist");
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
    double currentPosition = m_subsystem.wristMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("current Wrist position", currentPosition);
    SmartDashboard.putNumber("target Wrist position", rotationTarget);
    SmartDashboard.putBoolean("Wrist is going up", isGoingUp);
    System.out.println("executing move to Wrist");

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
    double currentPosition = m_subsystem.wristMotor.getEncoder().getPosition();
    if (isGoingUp) {
      if (currentPosition >= rotationTarget - 0.3) {
        return true;
      } else {
        return false;
      }
    } else {

      if (currentPosition <= rotationTarget + 0.3) {
        return true;
      } else {
        return false;
      }
    }

  }
}
