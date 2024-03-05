// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.MoveWristToPosCommand;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final AimSubsystem m_aimSubsystem = new AimSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  public DriverStation.Alliance teamColor;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  public Command Station_1_Shoot_Moveout;
  public Command Station_2_Shoot_Moveout;
  public Command Station_3_Shoot_Moveout;

  public Command moveForward;
  public SendableChooser<Command> m_chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> Constants.isRobotCentric,
            m_driverController.rightStick().getAsBoolean()
        ));

    m_launcherSubsystem.setDefaultCommand(new DefaultShooterCommand(m_launcherSubsystem));
    m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(m_elevatorSubsystem, m_operatorController));
    m_aimSubsystem.setDefaultCommand(new AimCommand(m_aimSubsystem, m_operatorController));

    // Configure bindings and limelight
    configureBindings();
    configureLimelight();
    configureAutoChooser();
    SmartDashboard.putData("Auto Mode", m_chooser);

  }

  private void configureBindings() {

    /*
     * 
     * TODO: Decide button bindings
     * 
     */

    // Scoring buttons
    m_operatorController.x().onTrue(ampLaunch());
    m_operatorController.y().onTrue(speakerLaunch());

    // Elevator buttons
    m_operatorController.a().onTrue(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator));

    // Wrist movement buttons
    m_operatorController.b().onTrue(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
    m_operatorController.leftTrigger().onTrue(new MoveWristToPosCommand(m_aimSubsystem, Constants.sourceAngle));

    // Index buttons
    m_operatorController.leftBumper().whileTrue(new IndexCommand(m_indexSubsystem, -0.2));
    m_operatorController.rightBumper().whileTrue(new IndexCommand(m_indexSubsystem, 0.2));

    // Intake command
    m_operatorController.rightTrigger().onTrue(new SourceIntakeCommand(m_indexSubsystem, m_launcherSubsystem));

    // Zero gyro button
    m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(), m_drivetrainSubsystem));
  }

  /*
   * Configures limelight to:
   * -Pipeline 0
   * -LEDs Off
   * -Proccesor Mode
   * -Pose relative to robot center (Meters and Degrees)
   */
  private void configureLimelight() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setCameraMode_Processor("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0, 0, 0, 0, 0, 0);
  }

  // Speaker launch sequential command
  public Command speakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        // TODO: Check wait time
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
  }

  // Amp launch sequential command
  public Command ampLaunch() {
    return new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.ampElevator)
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.ampTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        // TODO: Check wait time
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle))
        .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator));

  }

  // Climb sequence sequential command
  public Command climbSequence() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle)
        .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.climbingHeight))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.climbingAngle)
            .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator)));
  }

  // Creating different options for auto
  public void configureAutoChooser() {
    Station_1_Shoot_Moveout = speakerLaunch().andThen(new PathPlannerAuto("Station_1_Shoot_Moveout_Auto"));
    Station_2_Shoot_Moveout = speakerLaunch().andThen(new PathPlannerAuto("Station_2_Shoot_Moveout_Auto"));
    Station_3_Shoot_Moveout = speakerLaunch().andThen(new PathPlannerAuto("Station_3_Shoot_Moveout_Auto"));
  

    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Station 1", Station_1_Shoot_Moveout);
    m_chooser.addOption("Station 2", Station_2_Shoot_Moveout);
    m_chooser.addOption("Station 3", Station_3_Shoot_Moveout);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}