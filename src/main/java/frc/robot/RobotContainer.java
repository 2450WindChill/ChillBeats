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
import frc.robot.commands.CheckLauncherBeamBreak;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.WristLock;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.MoveIntakeToPosCommand;
import frc.robot.commands.MoveToPose;
// import frc.robot.commands.MoveIntakeToPosCommand;
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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public Alliance teamColor;
  private final LightySubsystem m_ledSubsystem = new LightySubsystem(this);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  private Command Center_5_Note_Middle_Note;
  private Command Center_5_Note_Top_Note;

  private Command Center_4_Note_All_Close;
  private Command Center_4_Note_All_Close_Rounded;
  private Command Center_4_Note_All_Far;
  private Command Center_4_Note_Close_Amp_Side_Far_Middle;
  private Command Center_4_Note_Close_Amp_Side_Far_Straight_Back;
  private Command Center_4_Note_Close_Amp_Side_Far_Top;
  private Command Center_4_Note_Close_Source_Side_Far_Middle;
  private Command Center_4_Note_Close_Source_Side_Far_Straight_Back;
  private Command Center_4_Note_Close_Source_Side_Far_Top;

  private Command Amp_Side_3_Note;
  private Command Center_3_Note_With_Middle;
  private Command Center_3_Note_With_Striaght_Back_Note;
  private Command Center_3_Note_With_Top_Note;
  private Command Source_Side_3_Note;
  private Command Center_3_Note_Rounded_Amp_Note;

  private Command Amp_Side_To_Amp_Side_Note;
  private Command Center_To_Amp_Side_Note;
  private Command Center_To_Center_Note;
  private Command Center_To_Source_Side_Note;
  private Command Source_Side_To_Source_Side_Note;
  private Command Source_Side_To_Bottom_Note;

  private Command Amp_Side_1_Note;
  private Command Source_Side_1_Note;
  private Command Center_1_Note;

  private Command Full_Disrupt_From_Amp_Side;
  private Command Full_Disrupt_From_Source_Side;

  private Command Test_Distance;

  private Command Shoot_Wait_Backup;

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
            m_driverController.leftTrigger()));

    // yes
    m_launcherSubsystem.setDefaultCommand(new DefaultShooterCommand(m_launcherSubsystem));
    // yes smaller
    m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(m_elevatorSubsystem, m_operatorController));
    m_aimSubsystem.setDefaultCommand(new AimCommand(m_aimSubsystem, m_operatorController));
    m_ledSubsystem.setDefaultCommand(new DefaultLEDCommand(m_launcherSubsystem, m_ledSubsystem));
    // m_indexSubsystem.setDefaultCommand(new DefaultIndexCommand(m_indexSubsystem,
    // m_ledSubsystem));

    // Configure bindings and limelight
    configureNamedCommands();
    configureBindings();
    configureLimelight();
    configureAutoChooser();
    SmartDashboard.putData("Auto Mode", m_chooser);

  }

  /*
   * Operator
   * x = amp prep
   * y = speaker shoot
   * right trigger = run shoot
   * left trigger = feeder in
   * Left Bumper = feeder out
   * Right Bumper = far launch
   * a = source intake
   * up dpad = zero arm
   * down dpad = turn off all motors
   */

  /*
   * Driver
   * x = zero gyro
   * b = ground intake
   */

  private void configureBindings() {
    // Operator
    m_operatorController.x().onTrue(ampLaunchPrep());
    m_operatorController.rightTrigger().onTrue(runShooter());
    m_operatorController.y().onTrue(autoSpeakerLaunch());
    m_operatorController.a().onTrue(sourceIntake());
    // m_operatorController.rightBumper().onTrue(farLaunchShoot());
    m_operatorController.povUp().onTrue(zeroArm());
    m_operatorController.povDown().onTrue(turnOffAllMotors());
    m_operatorController.b().onTrue(turnOffAllMotors().andThen(zeroArm()));

    m_operatorController.leftBumper()
        .onTrue(Commands.runOnce(() -> m_launcherSubsystem.feederOn(), m_launcherSubsystem))
        .onFalse(Commands.runOnce(() -> m_launcherSubsystem.feederOff()));
    m_operatorController.leftTrigger()
        .onTrue(Commands.runOnce(() -> m_launcherSubsystem.feederReverse(), m_launcherSubsystem))
        .onFalse(Commands.runOnce(() -> m_launcherSubsystem.feederOff()));
    // m_operatorController.rightBumper().onTrue(reverseIntake());

    // Driver
    m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(), m_drivetrainSubsystem));
    // m_driverController.b().onTrue(fullGroundIntake());
    m_driverController.povLeft().onTrue(totalReset());
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

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Shoot", autoSpeakerLaunch());
    NamedCommands.registerCommand("Intake", fullGroundIntake());
    NamedCommands.registerCommand("Instant Command", new InstantCommand());
  }

  public Command unstuckNote() {
    return Commands.runOnce(() -> m_launcherSubsystem.slowSpeakerTurnOnLauncher(), m_launcherSubsystem)
        .andThen(new WaitCommand(.1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOn(), m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff(), m_launcherSubsystem));
  }

  public Command reverseIntake() {
    return Commands.runOnce(() -> m_indexSubsystem.indexReverse())
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeReverse()))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederReverse()))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.indexOff()))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff()))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff()));
  }

  // Full speaker launch sequential command
  public Command autoSpeakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(),
        m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOn(),
            m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff(),
            m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(),
            m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem,
            Constants.zeroLaunchAngle));
  }

  public Command totalReset() {
    return turnOffAllMotors().andThen(zeroArm());
  }

  // Full speaker launch sequential command
  public Command farLaunchShoot() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(),
        m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOn(),
            m_launcherSubsystem))
        .andThen(new WaitCommand(2))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff(),
            m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(),
            m_launcherSubsystem));
  }

  // public Command fullGroundIntake() {

  // // Rev intake
  // return Commands.runOnce(() -> m_intakeSubsystem.intakeOn(),
  // m_intakeSubsystem)

  // // In parallel move intake down and turn on index
  // .andThen(Commands.parallel(new MoveIntakeToPosCommand(m_intakeSubsystem,
  // Constants.intakeDown),
  // // Turn on intake
  // Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem),
  // (new MoveWristToPosCommand(m_aimSubsystem, -8.1)))
  // // Turn on feeder
  // .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder())))
  // .andThen(new WaitCommand(1))
  // // Wait for index beam break to be trippedbane
  // .andThen(new CheckIndexBeamBreak(m_indexSubsystem))
  // //.andThen(new WaitCommand(.5))
  // .andThen(new MoveWristToPosCommand(m_aimSubsystem,
  // Constants.unstuckNoteAngle))
  // .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff(),
  // m_intakeSubsystem))
  // .andThen(new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.zeroIntake))
  // .andThen(new CheckLauncherBeamBreak(m_launcherSubsystem))
  // // .andThen(new MoveWristToPosCommand(m_aimSubsystem,
  // Constants.zeroLaunchAngle))
  // .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(),
  // m_launcherSubsystem))
  // .andThen(Commands.runOnce(() -> m_indexSubsystem.indexOff(),
  // m_indexSubsystem))
  // .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(),
  // m_launcherSubsystem));
  // }

  public Command fullGroundIntake() {

    // Rev intake
    return Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem)

        // In parallel move intake down and turn on index
        .andThen(Commands.parallel(new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.intakeDown)),
            Commands.runOnce(() -> m_launcherSubsystem.feederOn()),
            // Turn on intake
            Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem))
        .andThen(new WristLock(m_aimSubsystem, m_launcherSubsystem, -8.7))

        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem))

        .andThen(new CheckLauncherBeamBreak(m_launcherSubsystem))

        .andThen(Commands.runOnce(() -> m_launcherSubsystem.slowIntake()))
        .andThen(new WaitCommand(0.2))

        // Turn off all the motors
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff(), m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_indexSubsystem.indexOff(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        
        // Zero wrist and intake
        .andThen(Commands.parallel((new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.zeroIntake))),
            (new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle)));
  }

  public Command farLaunch() {

    return new MoveWristToPosCommand(m_aimSubsystem, Constants.farNoteLaunch)
        .andThen(farLaunchShoot())
        .andThen(zeroArm());

  }


  public Command testAllOn() {
    return Commands.parallel(
        Commands.runOnce(() -> m_launcherSubsystem.feederOn(), m_launcherSubsystem),
        Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem),
        Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem));
  }

  public Command testAllOff() {
    return Commands.parallel(
        Commands.runOnce(() -> m_launcherSubsystem.feederOff(), m_launcherSubsystem),
        Commands.runOnce(() -> m_indexSubsystem.indexOff(), m_indexSubsystem),
        Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem));
  }

  // Speaker launch w/ just wrist prep
  public Command speakerLaunchPrep() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle);
  }

  // // Amp launch with just elevator and wrist prep
  public Command ampLaunchPrep() {
    return Commands.parallel(
        new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.ampElevator),
        new MoveWristToPosCommand(m_aimSubsystem, Constants.ampAngle))
        .andThen(rumbleOperatorController(0.7));
  }

  // Shoot command
  public Command runShooter() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(), m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOn(), m_launcherSubsystem))
        .andThen(new WaitCommand(1.2))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff(), m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(), m_launcherSubsystem))
        .andThen(zeroArm())
        .andThen(Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  // Source intake
  public Command sourceIntake() {
    return new MoveWristToPosCommand(m_aimSubsystem, Constants.sourceAngle)
        .andThen(new SourceIntakeCommand(m_launcherSubsystem, m_ledSubsystem))
        .andThen(zeroArm())
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.feederOff()))
        .andThen(rumbleDriveController(0.7));
  }

  // Brings wrist and elevator to zero
  public Command zeroArm() {
    return Commands.parallel(Commands.parallel(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle),
        new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator),
        new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.intakeUp)));
  }

  public Command turnOffAllMotors() {
    return Commands.parallel(Commands.runOnce(() -> m_launcherSubsystem.feederOff()),
        Commands.runOnce(() -> m_indexSubsystem.indexOff()))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher())
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff())));
  }

  // Rumbles controller for a specified amount of time
  public Command rumbleDriveController(double duration) {
    return Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(duration))
        .andThen(Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .handleInterrupt(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  public Command rumbleOperatorController(double duration) {
    return Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(duration))
        .andThen(Commands.runOnce(() -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .handleInterrupt(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  // Returns our current alliance
  public Alliance getCurrentAliiance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  // Creating different options for auto
  public void configureAutoChooser() {

    // Center_5_Note_Middle_Note = new PathPlannerAuto("Center 5 Note Middle Note");
    // Center_5_Note_Top_Note = new PathPlannerAuto("Center 5 Note Top Note");

    // Center_4_Note_All_Close = new PathPlannerAuto("Center 4 Note All Close");
    Center_4_Note_All_Close_Rounded = new PathPlannerAuto("Center 4 Note All Close Rounded");
    Shoot_Wait_Backup = new PathPlannerAuto("Test_Auto");
    // Center_4_Note_All_Far = new PathPlannerAuto("Center 4 Note All Far");
    // Center_4_Note_Close_Amp_Side_Far_Middle = new PathPlannerAuto("Center 4 Note
    // Close Amp Side Far Middle");
    // Center_4_Note_Close_Amp_Side_Far_Straight_Back = new PathPlannerAuto("Center
    // 4 Note Close Amp Side Far Straight Back");
    // Center_4_Note_Close_Amp_Side_Far_Top = new PathPlannerAuto("Center 4 Note
    // Close Amp Side Far Top");
    // Center_4_Note_Close_Source_Side_Far_Middle = new PathPlannerAuto("Center 4
    // Note Close Source Side Far Middle");
    // Center_4_Note_Close_Source_Side_Far_Straight_Back = new
    // PathPlannerAuto("Center 4 Note Close Source Side Far Straight Back");
    // Center_4_Note_Close_Source_Side_Far_Top = new PathPlannerAuto("Center 4 Note
    // Close Source Side Far Top");

    // Amp_Side_3_Note = new PathPlannerAuto("Amp Side 3 Note");
    // Center_3_Note_With_Middle = new PathPlannerAuto("Center 3 Note With Middle");
    // Center_3_Note_With_Striaght_Back_Note = new PathPlannerAuto("Center 3 Note
    // With Striaght Back Note");
    // Center_3_Note_With_Top_Note = new PathPlannerAuto("Center 3 Note With Top
    // Note");
    // Source_Side_3_Note = new PathPlannerAuto("Source Side 3 Note");
    Center_3_Note_Rounded_Amp_Note = new PathPlannerAuto("Center 3 Note Rounded Amp Note");

    // Amp_Side_To_Amp_Side_Note = new PathPlannerAuto("Amp Side To Amp Side Note");
    // Center_To_Amp_Side_Note = new PathPlannerAuto("Center To Amp Side Note");
    Center_To_Center_Note = new PathPlannerAuto("Center To Center Note");
    // Center_To_Source_Side_Note = new PathPlannerAuto("Center To Source Side
    // Note");
    // Source_Side_To_Source_Side_Note = new PathPlannerAuto("Source Side To Source
    // Side Note");
    Source_Side_To_Bottom_Note = new PathPlannerAuto("Source Side To Bottom Note");

    Amp_Side_1_Note = new PathPlannerAuto("Amp Side 1 Note");
    Source_Side_1_Note = new PathPlannerAuto("Source Side 1 Note");
    Center_1_Note = new PathPlannerAuto("Center 1 Note");

    Full_Disrupt_From_Amp_Side = new PathPlannerAuto("Full Disrupt From Amp Side");
    Full_Disrupt_From_Source_Side = new PathPlannerAuto("Full Disrupt From Source Side");

    Test_Distance = new PathPlannerAuto("Test Distance");

    m_chooser = new SendableChooser<>();
    // m_chooser.addOption("Center_5_Note_Middle_Note", Center_5_Note_Middle_Note);
    // m_chooser.addOption("Center_5_Note_Top_Note", Center_5_Note_Top_Note);

    // m_chooser.addOption("Center_4_Note_All_Close", Center_4_Note_All_Close);
    m_chooser.addOption("Center_4_Note_All_Close_Rounded", Center_4_Note_All_Close_Rounded);
    m_chooser.addOption("Shoot, wait, backup", Shoot_Wait_Backup);
    // m_chooser.addOption("Center_4_Note_All_Far", Center_4_Note_All_Far);
    // m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Middle",
    // Center_4_Note_Close_Amp_Side_Far_Middle);
    // m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Straight_Back",
    // Center_4_Note_Close_Amp_Side_Far_Straight_Back);
    // m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Top",
    // Center_4_Note_Close_Amp_Side_Far_Top);
    // m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Middle",
    // Center_4_Note_Close_Source_Side_Far_Middle);
    // m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Straight_Back",
    // Center_4_Note_Close_Source_Side_Far_Straight_Back);
    // m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Top",
    // Center_4_Note_Close_Source_Side_Far_Top);

    // m_chooser.addOption("Amp_Side_3_Note", Amp_Side_3_Note);
    // m_chooser.addOption("Test auto", Center_To_Center_Note);
    // m_chooser.addOption("Center_3_Note_With_Middle", Center_3_Note_With_Middle);
    // m_chooser.addOption("Center_3_Note_With_Striaght_Back_Note",
    // Center_3_Note_With_Striaght_Back_Note);
    // m_chooser.addOption("Center_3_Note_With_Top_Note",
    // Center_3_Note_With_Top_Note);
    // m_chooser.addOption("Source_Side_3_Note", Source_Side_3_Note);
    m_chooser.addOption("Center_3_Note_Rounded_Amp_Note", Center_3_Note_Rounded_Amp_Note);

    // m_chooser.addOption("Amp_Side_To_Amp_Side_Note", Amp_Side_To_Amp_Side_Note);
    // m_chooser.addOption("Center_To_Amp_Side_Note", Center_To_Amp_Side_Note);
    m_chooser.addOption("Center_To_Center_Note", Center_To_Center_Note);
    // m_chooser.addOption("Center_To_Source_Side_Note",
    // Center_To_Source_Side_Note);
    // m_chooser.addOption("Source_Side_To_Source_Side_Note",
    // Source_Side_To_Source_Side_Note);
    m_chooser.addOption("Source_Side_To_Bottom_Note", Source_Side_To_Bottom_Note);

    m_chooser.addOption("Amp_Side_1_Note", Amp_Side_1_Note);
    m_chooser.addOption("Source_Side_1_Note", Source_Side_1_Note);
    m_chooser.addOption("Center_1_Note", Center_1_Note);

    m_chooser.addOption("Full_Disrupt_From_Amp_Side", Full_Disrupt_From_Amp_Side);
    m_chooser.addOption("Full_Disrupt_From_Source_Side", Full_Disrupt_From_Source_Side);

    m_chooser.addOption("Test_Distance", Test_Distance);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}