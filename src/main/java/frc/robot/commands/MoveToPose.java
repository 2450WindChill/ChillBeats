// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimate;

  public double currentX;
  public double currentY;
  public double currentRotation;

  public double desiredX;
  public double desiredY;
  public double desiredRotation;

  public double calculatedX;
  public double calculatedY;
  public double calculatedRotation;

  public Pose2d m_targetPose;

  public Translation2d speeds;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToPose(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimate, Pose2d targetPose) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimate = poseEstimate;
    m_targetPose = targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredX = m_targetPose.getX();
    desiredY = m_targetPose.getY();
    desiredRotation = m_targetPose.getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    calculatedX = desiredX - m_poseEstimate.getBotX();
    calculatedY = desiredY - m_poseEstimate.getBotY();
    calculatedRotation = desiredRotation - m_poseEstimate.getBotRotation();

    speeds = new Translation2d(
      calculatedX,
      calculatedY
    );

    m_drivetrainSubsystem.drive(
      speeds,
      calculatedRotation,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(
      new Translation2d(0, 0),
      0,
      false
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(calculatedRotation) <= 0.1) && (Math.abs(calculatedX) <= 0.1) && (Math.abs(calculatedY) <= 0.1)) {
      return true;
    } else {
      return false;
    }
  }
}
