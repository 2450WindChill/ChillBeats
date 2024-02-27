// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.MoveWristToPoseCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  private final ElevatorSubsystem m_ElevatorSubsystem = new
  ElevatorSubsystem();
  private final AimSubsystem m_aimSubsystem = new AimSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  public DriverStation.Alliance teamColor;
  // private final LightySubsystem m_LightySubsystem = new
  // LightySubsystem(teamColor);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  // public final JoystickButton operator_rightBumper = new
  // JoystickButton(m_driverController, Button.kRightBumper.value);
  // public final JoystickButton operator_lefttBumper = new
  // JoystickButton(m_driverController, Button.kRightBumper.value);
  // public final JoystickButton drive_yButton = new
  // JoystickButton(m_driverController, Button.kY.value);
  // public final JoystickButton buttonA = new JoystickButton(m_driverController,
  // Button.kA.value);

  public Command Station_1_Shoot_Moveout;
  public Command Station_2_Shoot_Moveout;
  public Command Station_3_Shoot_Moveout;
  public Command test;

  public Command moveForward;
  public SendableChooser<Command> m_chooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed

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
            () -> false));

    m_launcherSubsystem.setDefaultCommand(new DefaultShooterCommand(m_launcherSubsystem));

    // Configure bindings and limelight
    configureBindings();

    configureLimelight();
    configureAutoChooser();
    SmartDashboard.putData("Auto Mode", m_chooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(new MoveElevatorToPosCommand(m_ElevatorSubsystem, Constants.zeroElevator));
    m_driverController.x().onTrue(new MoveElevatorToPosCommand(m_ElevatorSubsystem, Constants.ampElevator));


    //TODO uncomment
   // m_operatorController.b().whileTrue(new IndexCommand(m_IndexSubsystem, -1));
    //m_operatorController.x().whileTrue(new IndexCommand(m_IndexSubsystem, 1));
    // m_operatorController.rightTrigger().whileTrue(new LaunchCommand(m_launcherSubsystem, -0.8));

    m_operatorController.leftTrigger().whileTrue(new LaunchCommand(m_launcherSubsystem, 0.2));

    m_driverController.a().onTrue(new MoveWristToPoseCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
    m_driverController.y().onTrue(new MoveWristToPoseCommand(m_aimSubsystem, Constants.speakerAngle));

    // Test buttons
    m_operatorController.x().onTrue(speakerLaunch());
    m_operatorController.b().onTrue(ampLaunch());
    m_driverController.b().onTrue(new TrajectoryCommand(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, new Pose2d()));

    /*
     * B -> Index for shoot
     * X -> Index for intake
     * ---------------------------------
     * Right trigger -> Launch for shoot
     * Left trigger -> Launch for intake
     * ---------------------------------
     * A -> zero angle
     * Y -> Launch angle
     */

  }

  /*
   * Configures limelight to:
   * -Pipeline 0
   * -LEDs Off
   * -Proccesor Mode
   * -Pose relative to robot center (Meters and Degrees)
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

  // public void setLEDsToAlliance() {
  // teamColor = DriverStation.getAlliance().get();
  // if (teamColor == DriverStation.Alliance.Red) {
  // System.err.println("Alliance RED");
  // m_LightySubsystem.SetLEDsToRed();
  // } else {
  // System.err.println("Alliance BLUE");
  // m_LightySubsystem.SetLEDsToBlue();
  // }

  // public void rainbow() {
  // m_LightySubsystem.rainbow();
  // }

  // public Command autoLaunch() {
  //   return (new MoveWristToPoseCommand(m_AimSubsystem, Constants.launchAngle))
  //       .HoldWristCommand()
  //       .alongWith(Commands.runOnce(() -> m_ShootSubsystem.turnOnLauncher(), m_ShootSubsystem))
  //       .andThen(new WaitCommand(3))
  //       .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOnIndexer(), m_IndexSubsystem))
  //       .andThen(new WaitCommand(2))
  //       .stophold
  //       .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOffIndexer(), m_IndexSubsystem))
  //       .andThen(Commands.runOnce(() -> m_ShootSubsystem.turnOffLauncher(), m_ShootSubsystem));
  // }

  public Command speakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPoseCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPoseCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
  }

    public Command ampLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.ampTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPoseCommand(m_aimSubsystem, Constants.ampAngle))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        .andThen(new WaitCommand(2))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPoseCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
  }

  
  public void configureAutoChooser() {
    Station_1_Shoot_Moveout = speakerLaunch();
    Station_2_Shoot_Moveout = speakerLaunch();
    Station_3_Shoot_Moveout = speakerLaunch();
    test = new PathPlannerAuto("test_auto");

    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Station 1", Station_1_Shoot_Moveout);
    m_chooser.addOption("Station 2", Station_2_Shoot_Moveout);
    m_chooser.addOption("Station 3", Station_3_Shoot_Moveout);

    m_chooser.addOption("Test Auto", test);
  }

  public Command getAutonomousCommand() {
    // return new InstantCommand();
    return m_chooser.getSelected();
    // return new PathPlannerAuto("SwerveAuto");
  }
}