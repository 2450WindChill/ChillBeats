// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WindChillSwerveModule;
import frc.robot.commands.TrajectoryCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

public class DrivetrainSubsystem extends SubsystemBase {
  public final Pigeon2 gyro;
  public WindChillSwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;
  public CANSparkMax testMotor;
  public CANcoder canCoder;
  public HolonomicDriveController holonomicDriveController;
  public TrajectoryCommand m_TrajectoryGeneratorSubsystem;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    holonomicDriveController = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));
        //SmartDashboard.putData("Reset pose", Commands.runOnce(() -> getX()));

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        PoseEstimatorSubsystem::getBotPose, // Robot pose supplier
        PoseEstimatorSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1, 0.2, 0.0), // Translation PID constants
            new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            Units.inchesToMeters(15.6875), // Drive base radius in meters. Distance from robot center to furthest
                                           // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    gyro = new Pigeon2(Constants.pigeonID);
    // gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    swerveModules = new WindChillSwerveModule[] {
        new WindChillSwerveModule(0, Constants.FrontLeftModule.constants),
        new WindChillSwerveModule(1, Constants.FrontRightModule.constants),
        new WindChillSwerveModule(2, Constants.BackLeftModule.constants),
        new WindChillSwerveModule(3, Constants.BackRightModule.constants)
    };
  }

  public void drive(Translation2d translation, double rotation, boolean isRobotCentric, boolean isSlowMode) {
    SwerveModuleState[] swerveModuleStates;
    // System.err.println("CALLING DRIVE");
    if (isRobotCentric) {
      // System.out.println("Angle value: " + translation.getY());
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    } else {
      // System.out.println("Angle value: " + translation.getY());
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              rotation,
              getGyroYaw()));
    }

    for (WindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isSlowMode);
    }
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
        path,
        PoseEstimatorSubsystem::getBotPose, // Robot pose supplier
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.driveKP, Constants.driveKI, Constants.driveKD), // Translation PID constants
            new PIDConstants(Constants.angleKP, Constants.angleKI, Constants.angleKD), // Rotation PID constants
            Constants.maxSpeed, // Max module speed, in m/s
            Constants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  public Rotation2d getGyroYaw() {
    Rotation2d rotation2d = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    return rotation2d;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (WindChillSwerveModule mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (WindChillSwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public WindChillSwerveModule[] getModules() {
    return swerveModules;
  }

  public void resetMods() {
    for (WindChillSwerveModule mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public double getFrontLeftEncoderVal() {
    double frontLeftEncoderVal = swerveModules[0].getDriveEncoder();

    return frontLeftEncoderVal;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void setGyro(int angle) {
    gyro.setYaw(angle);
  }

  // --------------------------------------------------------------

  // Pathplanning methods:

  /**
   * //Returns the heading of the robot.
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
  // public Pose2d getPose() {
  // return swerveOdometry.getPoseMeters();
  // }

  // public void resetPose(Pose2d pose2d) {
  // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), getPose());
  // }

  public StatusCode resetCANcoder() {
    return canCoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetCANcoder();
    swerveOdometry.resetPosition(
        gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxSpeed);
    var modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i], false);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    SwerveModuleState[] modStates = Constants.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);

    for (WindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(modStates[mod.moduleNumber], false);
    }
  }
  // -----------------------------------------------------------------------------------------------

  
  public void setPosition(double position) {
    swerveModules[0].setPosition(position);
    System.err.println("Setting position to 0");
  }

  public void motorAuto() {
    System.err.println("Motor Auto go");
    drive(new Translation2d(-0.8, 0), gyro.getYaw().getValue(), true, false);
  }

  public void stopAuto() {
    System.err.println("Motor auto stop");
    drive(new Translation2d(0, 0), gyro.getYaw().getValue(), true, false);
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
    }

    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Roll", gyro.getRoll().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {

  }
}