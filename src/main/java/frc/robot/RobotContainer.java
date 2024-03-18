// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FieldCentricAutoDrive;
import frc.robot.commands.CheckIndexBeamBreak;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.LockMoveWristToPosCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.MoveIntakeToPosCommand;
import frc.robot.commands.MoveWristToPosCommand;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  // private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  public Alliance teamColor;
  private final LightySubsystem m_ledSubsystem = new LightySubsystem(this);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  public Command Blue_Short_Side;
  public Command Blue_Middle_Side;
  public Command Blue_Long_Side;

  public Command Red_Short_Side;
  public Command Red_Middle_Side;
  public Command Red_Long_Side;

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
            () -> m_driverController.rightStick().getAsBoolean()));

    m_launcherSubsystem.setDefaultCommand(new DefaultShooterCommand(m_launcherSubsystem));
    m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(m_elevatorSubsystem, m_operatorController));
    m_aimSubsystem.setDefaultCommand(new AimCommand(m_aimSubsystem, m_operatorController));
    m_ledSubsystem.setDefaultCommand(new DefaultLEDCommand(m_launcherSubsystem, m_ledSubsystem));
    // m_indexSubsystem.setDefaultCommand(new DefaultIndexCommand(m_indexSubsystem,
    // m_ledSubsystem));

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
    m_operatorController.y().onTrue(autoSpeakerLaunch());
    m_operatorController.a().onTrue(sourceIntake());
    m_operatorController.b().onTrue(zeroArm());
    m_operatorController.leftTrigger().whileTrue(new LaunchCommand(m_launcherSubsystem, -0.3));
    m_operatorController.leftTrigger().whileTrue(new IndexCommand(m_launcherSubsystem, .1));
    // m_operatorController.leftBumper().onTrue(groundIntake());

    // Driver
    // Zero Gyro
    m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(), m_drivetrainSubsystem));
    m_driverController.b().onTrue(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.maxHeight));

    // TEMPORARY TESTING
    // m_driverController.leftTrigger().whileTrue(Commands.runOnce(() ->
    // m_intakeSubsystem.intakeOn(), m_intakeSubsystem)
    // .finallyDo(() -> m_intakeSubsystem.intakeOff()));
    // m_driverController.rightTrigger().whileTrue(Commands.runOnce(() ->
    // m_indexSubsystem.indexOn(), m_indexSubsystem)
    // .finallyDo(() -> m_indexSubsystem.indexOff()));
  }

  /*
   * Configures limelight to:
   * -Pipeline 0
   * -LEDs Off
   * -Proccesor Mo
   * -Pose relative to robot center (Meters and Degrees)
   */
  private void configureLimelight() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setCameraMode_Processor("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0, 0, 0, 0, 0, 0);
  }

  // Full speaker launch sequential command
  public Command autoSpeakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(), m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle));
  }

  // public Command groundIntake() {

  // return Commands.parallel(
  // Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem),
  // Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem),
  // // CHANGE THIS NUMBER LATER IT SHOULDNT BE 5 TEST IT
  // new MoveIntakeToPosCommand(m_intakeSubsystem, 5.1))

        .andThen(new CheckIndexBeamBreak(m_indexSubsystem))

        .andThen(Commands.parallel(
            Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem),
            Commands.runOnce(() -> m_indexSubsystem.indexOff(), m_launcherSubsystem),
            // CHANGE THIS NUMBER LATER IT SHOULDNT BE 5 TEST IT
            new MoveIntakeToPosCommand(m_intakeSubsystem, 0.0)));
  }

  // Speaker launch w/ just wrist prep
  public Command speakerLaunchPrep() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle);
  }

  // Amp launch with just elevator and wrist prep
  public Command ampLaunchPrep() {
    return Commands.parallel(
        new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.ampElevator),
        new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle))
        .andThen(rumbleOperatorController(0.7));
  }

  // Shoot command
  public Command shoot() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem))
        .andThen(new WaitCommand(.3))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(), m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(zeroArm())
        .andThen(Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  // Source intake
  public Command sourceIntake() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.sourceAngle)
        .andThen(new SourceIntakeCommand(m_launcherSubsystem))
        .andThen(rumbleDriveController(0.7))
        .andThen(zeroArm());
  }

  // Brings wrist and elevator to zero
  public Command zeroArm() {
    return Commands.parallel(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle),
        new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator));
  }

  // Rumbles controller for a specified amount of time
  public Command rumbleDriveController(double duration) {
    return Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(duration))
        .andThen(Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command rumbleOperatorController(double duration) {
    return Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(duration))
        .andThen(Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  // Returns our current alliance
  public Alliance getCurrentAliiance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  // Climb sequence sequential command
  // public Command climbSequence() {
  // return new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle)
  // .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem,
  // Constants.climbingHeight))
  // .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.climbingAngle)
  // .andThen(new MoveElevatorToPosCommand(m_elevatorSubsystem,
  // Constants.zeroElevator)));
  // }

  // Creating different options for auto
  public void configureAutoChooser() {

    Blue_Short_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(230), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new WaitCommand(4))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, -1.5), 0))
        .andThen(new WaitCommand(.5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(-1.5, 0), 0))
        .andThen(new WaitCommand(5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    Blue_Middle_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(180), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(-1.5, 0), 0))
        .andThen(new WaitCommand(2))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    Blue_Long_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(144), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new WaitCommand(3.5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 1.5), 0))
        .andThen(new WaitCommand(1))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(-1.5, 0), 0))
        .andThen(new WaitCommand(5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    Red_Short_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(230), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, -1.5), 0))
        .andThen(new WaitCommand(.5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(1.5, 0), 0))
        .andThen(new WaitCommand(5))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    Red_Middle_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(180), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(-1.5, 0), 0))
        .andThen(new WaitCommand(2))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    Red_Long_Side = Commands.runOnce(() -> m_drivetrainSubsystem.setGyro(144), m_drivetrainSubsystem)
        .andThen(autoSpeakerLaunch())
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(-1.5, 0), 0))
        .andThen(new WaitCommand(1.85))
        .andThen(new FieldCentricAutoDrive(m_drivetrainSubsystem, new Translation2d(0, 0), 0));

    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Blue Stage Side", Blue_Middle_Side);
    m_chooser.addOption("Blue Amp Side", Blue_Short_Side);
    m_chooser.addOption("Blue Source Side", Blue_Long_Side);
    m_chooser.addOption("Red Amp Side", Red_Short_Side);
    m_chooser.addOption("Red Stage Side", Red_Middle_Side);
    m_chooser.addOption("Red Source Side", Red_Long_Side);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
