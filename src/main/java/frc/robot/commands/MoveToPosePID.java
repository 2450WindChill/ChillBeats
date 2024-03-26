// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPosePID extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimate;

  public double currentX;
  public double currentY;
  public double currentRotation;

  public double calculatedX;
  public double calculatedY;
  public double calculatedRotation;

  public double finalXSpeed;
  public double finalYSpeed;
  public double finalRotation;

  public Pose2d m_targetPose;

  public Translation2d speeds;

  PIDController xPIDController = new PIDController(0.1, 0.1, 0);
  PIDController yPIDController = new PIDController(0.1, 0.1, 0);
  PIDController rotationPIDController = new PIDController(0.1, 0.1, 0.0036);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToPosePID(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimate, Pose2d targetPose) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimate = poseEstimate;
    m_targetPose = targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPIDController.reset();
    yPIDController.reset();
    rotationPIDController.reset();

    xPIDController.setSetpoint(m_targetPose.getX());
    xPIDController.setSetpoint(m_targetPose.getY());
    rotationPIDController.setSetpoint(m_targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Finds translation and rotation to desired pose
    System.out.println("MOVE TO POSE");
    calculatedX = xPIDController.calculate(m_poseEstimate.getBotX());
    calculatedY = yPIDController.calculate(m_poseEstimate.getBotY());
    calculatedRotation = rotationPIDController.calculate(m_poseEstimate.getBotRotation());

    MathUtil.clamp(calculatedX, -1, 1);
    MathUtil.clamp(calculatedY, -1, 1);
    MathUtil.clamp(calculatedRotation, -1, 1);

    SmartDashboard.putNumber("Calculated X", calculatedX);
    SmartDashboard.putNumber("Calculated Y", calculatedY);
    SmartDashboard.putNumber("Calculated Rotation", calculatedRotation);


    // Calls .drive() with speeds and rotations towards desired pose
    m_drivetrainSubsystem.drive(
      new Translation2d(calculatedX, calculatedY),
      calculatedRotation,
      false,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(
      new Translation2d(0, 0),
      0,
      false,
      false
    );


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(calculatedRotation) <= 20) && (Math.abs(calculatedX) <= 0.1) && (Math.abs(calculatedY) <= 0.1)) {
      System.out.println("MOVE TO POSE IS FINISHED");
      return true;
    } else {
      System.err.println("move to pose NOT finished");
      return false;
    }
  }
}
