package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class FieldCentricAutoDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_driveSubsystem;
  private double currentLocationFeet;
  private double targetLocationFeet;

  private Translation2d m_speeds;
  private double m_rotation;

  private boolean movingForward;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldCentricAutoDrive(DrivetrainSubsystem drivetrainSubsystem, Translation2d speeds, double rotation) {
    m_driveSubsystem = drivetrainSubsystem;
    m_speeds = speeds;
    m_rotation = rotation;

    // targetLocationFeet = desiredDistanceFeet + currentLocationFeet;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.err.println("INITIALIZE");
    // currentLocationFeet = Math.abs(m_driveSubsystem.getFrontLeftEncoderVal() / Constants.rotationsPerOneFoot);

    // if (currentLocationFeet < targetLocationFeet) {
    //   movingForward = true;
    // } else {
    //   movingForward = false;
    // }

    m_driveSubsystem.drive(m_speeds, m_rotation, false, false);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.err.println("EXECUTING");
    // double currentLocationRotations = m_driveSubsystem.getFrontLeftEncoderVal();
    // currentLocationFeet = Math.abs(currentLocationRotations / Constants.rotationsPerOneFoot);

    // SmartDashboard.putNumber("Target Location", targetLocationFeet);
    // SmartDashboard.putNumber("Current Location", currentLocationFeet);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_driveSubsystem.drive(new Translation2d(0, 0), m_driveSubsystem.gyro.getYaw(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return true;
  }
}

