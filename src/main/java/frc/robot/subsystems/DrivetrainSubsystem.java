// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WindChillSwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  public final Pigeon2 gyro;
  private WindChillSwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;
  public CANSparkMax testMotor;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {

    gyro = new Pigeon2(Constants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    swerveModules = new WindChillSwerveModule[] {
      new WindChillSwerveModule(0, Constants.FrontLeftModule.constants),
      new WindChillSwerveModule(1, Constants.FrontRightModule.constants),
      new WindChillSwerveModule(2, Constants.BackLeftModule.constants),
      new WindChillSwerveModule(3, Constants.BackRightModule.constants)
    };

    swerveOdometry = new SwerveDriveOdometry(
      Constants.swerveKinematics,
      getGyroAsRotation2d(),
      getModulePositions(),
      new Pose2d(0, 0, new Rotation2d(0))
    );

    periodic();
  }

  // TODO: Figure out these encoders
    // The left-side drive encoder
    private final Encoder m_leftEncoder =
    new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
    new Encoder(
        DriveConstants.kRightEncoderPorts[0],
        DriveConstants.kRightEncoderPorts[1],
        DriveConstants.kRightEncoderReversed);

  public void drive(Translation2d translation, double rotation, boolean isRobotCentric) {
    SwerveModuleState[] swerveModuleStates;
    // System.err.println("CALLING DRIVE");
    if (isRobotCentric) {
      // System.out.println("Angle value: " + translation.getY());
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
      );
    } else {
      // System.out.println("Angle value: " + translation.getY());
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), 
          translation.getY(), 
          rotation, 
          getGyroAsRotation2d())
      );
    }

    for (WindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
    }
  }

  // public void autonomousDrive(double xSpeed, double ySpeed, double rotation) {
  //   SwerveModuleState[] swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
  //       new ChassisSpeeds(xSpeed, ySpeed, rotation)
  //     );

  //   for (WindChillSwerveModule mod : swerveModules) {
  //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
  //   }
  // }

  // public void fieldCentricAutonomousDrive(double xSpeed, double ySpeed, double rotation) {
  //   SwerveModuleState[] swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
  //       ChassisSpeeds.fromFieldRelativeSpeeds(
  //         xSpeed,
  //         ySpeed,
  //         rotation,
  //         getGyroAsRotation2d())
  //     );

  //   for (WindChillSwerveModule mod : swerveModules) {
  //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
  //   }
  // }

  public Rotation2d getGyroAsRotation2d() {
    Rotation2d rotation2d = Rotation2d.fromDegrees(gyro.getYaw().getValue());

    return rotation2d;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(WindChillSwerveModule mod : swerveModules){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public double getFrontLeftEncoderVal(){
      double frontLeftEncoderVal = swerveModules[0].getDriveEncoder();
      
      return frontLeftEncoderVal;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  // --------------------------------------------------------------

  // Pathplanning methods:

    /**
    //Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */

   // TODO: Figure out motor groups and volts constants
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  // TODO: change encoders
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  // TODO: Change for swerve
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

// -----------------------------------------------------------------------------------------------

  public void setPosition(double position) {
    swerveModules[0].setPosition(position);
    System.err.println("Setting position to 0");
  }

  public void motorAuto() {
    System.err.println("Motor Auto go");
    drive(new Translation2d(-0.8, 0), gyro.getYaw().getValue(), true);
  }

  public void stopAuto() {
    System.err.println("Motor auto stop");
    drive(new Translation2d(0, 0), gyro.getYaw().getValue(), true);
  }

  @Override
  public void periodic() {
    for (WindChillSwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getRawCanCoder());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      
      // TODO: Change with new encoders
      odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

  

    swerveOdometry.update(
      getGyroAsRotation2d(),
      getModulePositions()
    );
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().getValue());
    SmartDashboard.putNumber("Gyro Roll", gyro.getRoll().getValue());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch().getValue());


  }

  @Override
  public void simulationPeriodic() {

  }
}