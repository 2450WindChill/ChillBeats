// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIndexCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.LockMoveWristToPosCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.MoveWristToPosCommand;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public Alliance teamColor = DriverStation.getAlliance().orElse(Alliance.Blue);
  private final LightySubsystem m_ledSubsystem = new LightySubsystem(teamColor);

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
    m_indexSubsystem.setDefaultCommand(new DefaultIndexCommand(m_indexSubsystem, m_ledSubsystem));

    // Configure bindings and limelight
    configureBindings();
    configureLimelight();
    configureAutoChooser();
    SmartDashboard.putData("Auto Mode", m_chooser);

  }

  /*
   * x = amp
   * y = speaker
   * triggers = index
   * a = source
   * b = zero arm
   */

  private void configureBindings() {

    // Operator
      // Sequences
    m_operatorController.x().onTrue(ampLaunchPrep());
    m_operatorController.rightTrigger().onTrue(shoot());
    m_operatorController.y().whileTrue(new LockMoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle));
    m_operatorController.a().onTrue(sourceIntake());
    m_operatorController.b().onTrue(zeroArm());

    // Driver
      // Zero Gyro
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
  public Command autoSpeakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
  }

  public Command speakerLaunchPrep() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle);
  }

  public Command ampLaunchPrep() {
    return (new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.ampElevator))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle));
  }

  public Command shoot() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOnIndexer(), m_indexSubsystem))
        .andThen(new WaitCommand(.3))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.turnOffIndexer(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(zeroArm());
  }

 
  public Command sourceIntake() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.sourceAngle)
        .andThen(new SourceIntakeCommand(m_indexSubsystem, m_launcherSubsystem));
  }

  public Command zeroArm() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle)
        .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator));
  }

  // Climb sequence sequential command
  // public Command climbSequence() {
  //   return new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle)
  //       .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.climbingHeight))
  //       .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.climbingAngle)
  //           .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator)));
  // }

  // Creating different options for auto
  public void configureAutoChooser() {
    Station_1_Shoot_Moveout = autoSpeakerLaunch()/*.andThen(new PathPlannerAuto("Station_1_Shoot_Moveout_Auto"))*/;
    Station_2_Shoot_Moveout = autoSpeakerLaunch()/*.andThen(new PathPlannerAuto("Station_2_Shoot_Moveout_Auto"))*/;
    Station_3_Shoot_Moveout = autoSpeakerLaunch()/*.andThen(new PathPlannerAuto("Station_3_Shoot_Moveout_Auto"))*/;
  

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