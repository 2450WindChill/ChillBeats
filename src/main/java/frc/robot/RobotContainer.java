// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveToPose;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  

   static XboxController m_driverController = new XboxController(0);
  static XboxController m_operatorController = new XboxController(1);

  public final JoystickButton drive_rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  public final JoystickButton drive_yButton = new JoystickButton(m_driverController, Button.kY.value);
  public final JoystickButton buttonA = new JoystickButton(m_driverController, Button.kA.value);

  public Command moveForward;

  private SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> true
          ));

    // Configure bindings and limelight
    configureBindings();
    configureAutoChooser();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureLimelight();
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive_yButton.onTrue(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d()));
  }

  public void configureAutoChooser() {
    moveForward = Commands.runOnce(() -> new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0.2, 0, new Rotation2d(0)))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(-0.2, 0, new Rotation2d(0))))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, -0.2, new Rotation2d(0))))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, 0.2, new Rotation2d(0)))));

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Test Auto", moveForward);
  }
                   

  /*
   * Configures limelight to:
   *  -Pipeline 0
   *  -LEDs Off
   *  -Proccesor Mode
   *  -Pose relative to robot center (Meters and Degrees)
   */
  private void configureLimelight() {
    LimelightHelpers.setPipelineIndex("limelights", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setCameraMode_Processor("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0, 0, 0, 0, 0, 0);
  }
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */   
  public Command getAutonomousCommand() {
    System.out.println("AUTO IS RUNNING");
     return new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0.2, 0, new Rotation2d(0)))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(-0.2, 0, new Rotation2d(0))))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, -0.2, new Rotation2d(0))))
    .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, 0.2, new Rotation2d(0))));
    //  return new PathPlannerAuto("SwerveAuto");
    //return autoChooser.getSelected();
}
    // // Example auto which moves the robot forward 5 meters and then back to the origin without rotating
    // return new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(5, 0, new Rotation2d(0)))
    //           .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d()));
  }
