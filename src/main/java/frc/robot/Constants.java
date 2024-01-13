// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.55;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.54;

  // TODO: Fine tune this value
  public static final double stickDeadband = 0.125;

  public static final int pigeonID = 13;
  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  /* Drivetrain Constants */
  public static final double trackWidth = Units.inchesToMeters(21.73);
  public static final double wheelBase = Units.inchesToMeters(21.73);
  public static final double wheelDiameter = Units.inchesToMeters(4.0);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;
  

  public static final double rotationsPerOneFoot = 0.33;

  // TODO : Configure ratios (look up)
  public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1
  public static final double angleGearRatio = (150 / 7); // 12.8:1

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      // Front Left
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      // Front right
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      // Back left
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      // Back right
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

  /* Swerve Voltage Compensation */
  public static final double voltageComp = 12.0;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 20;
  public static final int driveContinuousCurrentLimit = 80;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.01;
  public static final double angleKI = 0.0;
  public static final double angleKD = 0.0;
  public static final double angleKFF = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.1;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKFF = 0.0;

  /* Drive Motor Conversion Factors */
  public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
  public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
  public static final double angleConversionFactor = 360.0 / angleGearRatio;

  /* Swerve Profiling Values */
  //public static final double maxSpeed = 3.6576; // meters per second

  // Max speed is 4.630
  public static final double maxSpeed = 6380 / 60 * ModuleConfiguration.MK4I_L1.getDriveReduction() * ModuleConfiguration.MK4I_L1.getWheelDiameter() * Math.PI; 

  //TODO: Test rotation

  // 0.2?
  public static final double maxAngularVelocity = maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);;

  public static final double TICKS_PER_ROTATION = 42;

  /* Neutral Modes */
  // public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  // public static final IdleMode driveNeutralMode = IdleMode.kBrake;

  /* Motor Inverts */

  // TODO: Figure out value
  public static final boolean driveInvert = false;
  public static final boolean angleInvert = true;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = false;

  public static final double aimTolerance = 4.5;

  //TODO: Add offsets 
  /* Front Left Module - Module 0 */
  public static final class FrontLeftModule {
    public static final int driveMotorID = 7;
    public static final int angleMotorID = 8;
    public static final int canCoderID = 12;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.176);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRightModule {
    public static final int driveMotorID = 5;
    public static final int angleMotorID = 6;
    public static final int canCoderID = 11;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-.361);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeftModule {
    public static final int driveMotorID = 1;
    public static final int angleMotorID = 2;
    public static final int canCoderID = 9;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.206);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRightModule {
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 10;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.381);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
