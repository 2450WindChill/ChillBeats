// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.libs.ModuleConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
     public static final int kOperatorControllerPort = 1;
  }
  
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

  public static final double speakerAngle = -8.0;
  public static final double ampAngle = -41.14;
  public static final double zeroLaunchAngle = -1.5;

  public static final double zeroElevator = 0.0;
  public static final double ampElevator = 60;

  public static final double sourceAngle = -17.1;
  public static final double sourceHeight = 0.0;

  public static final double maxHeight = 127;
  public static final double climbingAngle = -24.6;
  

  public static final double rotationsPerOneFoot = 0.33;
  public static final double feetToMeters = 0.3048;

  public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1
  public static final double angleGearRatio = (150 / 7); // 12.8:1

  public static final double driveBaseRadius = (Math.sqrt((trackWidth * trackWidth) + (trackWidth * trackWidth)))/2;

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
  public static final int driveContinuousCurrentLimit = 30;

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
  public static final double angleConversionFactor = 360.0 /* (2 * Math.PI) */ / angleGearRatio;

  /* Swerve Profiling Values */
  //public static final double maxSpeed = 3.6576; // meters per second

  // Max speed is 4.630
  public static final double maxSpeed = 6380 / 60 * ModuleConfiguration.MK4I_L1.getDriveReduction() * ModuleConfiguration.MK4I_L1.getWheelDiameter() * Math.PI; 

  public static final double maxAngularVelocity = maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);;

  public static final double TICKS_PER_ROTATION = 42;

  /* Neutral Modes */
  // public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final IdleMode angleBrakeMode = IdleMode.kBrake;
  public static final IdleMode driveBrakeMode = IdleMode.kBrake;
  public static final IdleMode elevatorBrakeMode = IdleMode.kBrake;

  /* Motor Inverts */

  public static final boolean driveInvert = false;
  public static final boolean angleInvert = true;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = false;

  /* Robot centric invert */
  public static final boolean isRobotCentric = false;

  /* Front Left Module - Module 0 */
  public static final class FrontLeftModule {
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 5;
    public static final int canCoderID = 11;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRightModule {
    public static final int driveMotorID = 8;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 12;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeftModule {
    public static final int driveMotorID = 4;
    public static final int angleMotorID = 3;
    public static final int canCoderID = 10;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRightModule {
    public static final int driveMotorID = 2;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 9;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }
  

  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  public static final double kPDriveVel = 8.5;

  public static final double autoVoltageConstraint = 0;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kPThetaController = 1;
  public static final double moveToPoseSpeed = 0.05;
  public static final double moveToPoseRotationSpeed = 0.5;

  public static final class BlueAllianceFieldElementsPoses {
    public static Pose2d source;
    public static Pose2d amp;
    public static Pose2d speaker;
  }

  public static final class RedAllianceFieldElementsPoses {
    public static Pose2d source;
    public static Pose2d amp;
    public static Pose2d speaker;
  }

  // In meters
  public static final int[] distancesFromSpeaker = {0,1,2,3,4,5,6};

  // In encoder values
  // IDS HERE MATCH WITH IDS IN DISTANCESfROMsPEAKER
  public static final double[] anglesToShootInSpeaker = {0, 0, -13.85};
}
