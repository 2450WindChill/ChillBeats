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
import frc.robot.commands.IndexCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.LockMoveWristToPosCommand;
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

  private Command Amp_Side_To_Amp_Side_Note;
  private Command Center_To_Amp_Side_Note;
  private Command Center_To_Center_Note;
  private Command Center_To_Source_Side_Note;
  private Command Source_Side_To_Source_Side_Note;

  private Command Full_Disrupt_From_Amp_Side;
  private Command Full_Disrupt_From_Source_Side;

  private Command Test_Auto;

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
            () -> m_driverController.leftTrigger().getAsBoolean()));

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
    configureNamedCommands();
    SmartDashboard.putData("Auto Mode", m_chooser);

  }

  /*
   * x = amp
   * y = speaker
   * right trigger = shoot
   * left trigger = feeder in
   * Left Bumper = unstucknote
   * a = source
   * b = ground intake
   * up dpad = zero
   * 
   */

  private void configureBindings() {

    // Operator
    // Sequences
    m_operatorController.x().onTrue(ampLaunchPrep());
    m_operatorController.rightTrigger().onTrue(shoot());
    m_operatorController.y().onTrue(autoSpeakerLaunch());
    m_operatorController.a().onTrue(sourceIntake());
    m_operatorController.b().onTrue(fullGroundIntake());
    m_operatorController.leftBumper().onTrue(unstuckNote());
    m_operatorController.leftTrigger().onTrue(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem)).onFalse(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder()));

    // m_operatorController.leftBumper().onTrue(Commands.runOnce(() -> m_launcherSubsystem.slowSpeakerTurnOnLauncher()))
    //  .onFalse(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher()));

     m_operatorController.povUp().onTrue(zeroArm());
     // Intake on
    m_driverController.a().onTrue(Commands.runOnce(() -> m_intakeSubsystem.intakeOn())).onFalse(Commands.runOnce(() -> m_intakeSubsystem.intakeOff()));
    // Angle intake
    m_driverController.y().onTrue(Commands.runOnce(() -> m_intakeSubsystem.angleIntakeOn())).onFalse(Commands.runOnce(() -> m_intakeSubsystem.angleIntakeOff()));

   
    // Index on
    m_driverController.b().onTrue(Commands.runOnce(() -> m_indexSubsystem.indexOn())).onFalse(Commands.runOnce(() -> m_indexSubsystem.indexOff()));

    m_driverController.povUp().onTrue(testAllOn()).onFalse(testAllOff());

    // m_operatorController.leftTrigger().whileTrue(new
    // IndexCommand(m_launcherSubsystem, .1));

    // m_operatorController.leftTrigger().onTrue(fullGroundIntake());
    // m_operatorController.rightTrigger().onTrue(partialGroundIntake());

    m_driverController.leftTrigger().onTrue(new
    MoveToPose(m_drivetrainSubsystem, m_poseEstimator,
    new Pose2d(new Translation2d(1, 0), new Rotation2d())));

    m_driverController.rightTrigger().onTrue(new
    MoveToPose(m_drivetrainSubsystem, m_poseEstimator,
    new Pose2d(new Translation2d(0, 0), new Rotation2d())));

    // Driver
    // Zero Gyro
    // TODO: uncommment
    //m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(), m_drivetrainSubsystem));
   // m_driverController.b().onTrue(new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.maxHeight));

    // TEMPORARY TESTING
    // m_driverController.leftTrigger().whileTrue(Commands.runOnce(() ->
    // m_intakeSubsystem.intakeOn(), m_intakeSubsystem))
    // .onFalse(Commands.runOnce(() -> m_intakeSubsystem.intakeOff()));
    // m_driverController.rightTrigger().whileTrue(Commands.runOnce(() ->
    // m_indexSubsystem.indexOn(), m_intakeSubsystem))
    // .onFalse(Commands.runOnce(() -> m_indexSubsystem.indexOff()));
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
  }

 public Command unstuckNote() {
    return Commands.runOnce(() -> m_launcherSubsystem.slowSpeakerTurnOnLauncher(), m_launcherSubsystem)
    .andThen(new WaitCommand(.1))
    .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem))
    .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(), m_launcherSubsystem));
  }


  // Full speaker launch sequential command
  public Command autoSpeakerLaunch() {
    return (Commands.runOnce(() -> m_launcherSubsystem.speakerTurnOnLauncher(),
        m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(),
            m_launcherSubsystem))
        .andThen(new WaitCommand(1))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(),
            m_launcherSubsystem))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffLauncher(),
            m_launcherSubsystem))
        .andThen(new MoveWristToPosCommand(m_aimSubsystem,
            Constants.zeroLaunchAngle));
  }

 
     
  
  // // Full speaker launch sequential command
  // public Command autoSpeakerLaunch() {
  // return Commands.parallel(
  // // Shoot and angle
  // new MoveWristToPosCommand(m_aimSubsystem, Constants.speakerAngle),
  // // Turn on launcher and feeder
  // Commands.runOnce(() -> m_launcherSubsystem.turnOnLauncherAndFeeder(),
  // m_launcherSubsystem))
  // // Wait
  // .andThen(new WaitCommand(1))
  // // Turn off feeder and launcher and zero wrist
  // .andThen(Commands.parallel(
  // Commands.runOnce(() -> m_launcherSubsystem.turnOffFeederAndLauncher(),
  // m_launcherSubsystem),
  // new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle)
  // ));
  // }

  public Command partialGroundIntake() {
    return Commands.parallel(
        // Flip out and turn on intake
        Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder()).andThen(
        new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.intakeDown),
        // Turn on intake
        Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem))

        // Wait for index beam break to be tripped
        .andThen(new CheckIndexBeamBreak(m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem))
        .andThen(Commands.parallel(

            new MoveIntakeToPosCommand(m_intakeSubsystem, 0.0),
            // Turn on feeder
            Commands.runOnce(() -> m_indexSubsystem.indexOff(), m_indexSubsystem))));
  }

  public Command fullGroundIntake() {
    return Commands.parallel(
        // Flip out and turn on intake
        // Turn on feeder
        Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem),
      //  new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.intakeDown),
        // Turn on intake
        Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem))

        // Wait for index beam break to be tripped
        .andThen(new CheckIndexBeamBreak(m_indexSubsystem))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem))
        .andThen(Commands.parallel(

            new MoveIntakeToPosCommand(m_intakeSubsystem, 0.0),
            // Wait for wrist beam break to be tripped
            new CheckLauncherBeamBreak(m_launcherSubsystem))

            .andThen(Commands.parallel(
                // Turn off indexer and feeder
                Commands.runOnce(() -> m_indexSubsystem.indexOff())
                    .andThen(() -> m_launcherSubsystem.turnOffFeeder())))

        );

  }

  public Command testAllOn() {
    return Commands.parallel(
        Commands.runOnce(() -> m_launcherSubsystem.turnOnFeeder(), m_launcherSubsystem),
        Commands.runOnce(() -> m_indexSubsystem.indexOn(), m_indexSubsystem),
        Commands.runOnce(() -> m_intakeSubsystem.intakeOn(), m_intakeSubsystem));
  }

  public Command testAllOff() {
    return Commands.parallel(
        Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder(), m_launcherSubsystem),
        Commands.runOnce(() -> m_indexSubsystem.indexOff(), m_indexSubsystem),
        Commands.runOnce(() -> m_intakeSubsystem.intakeOff(), m_intakeSubsystem));
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
        .andThen(new WaitCommand(.15))
        .andThen(Commands.runOnce(() -> m_launcherSubsystem.turnOffFeeder()))
        .andThen(rumbleDriveController(0.7))
        .andThen(zeroArm());
  }

  

  // Brings wrist and elevator to zero
  public Command zeroArm() {
    return Commands.parallel(new MoveWristToPosCommand(m_aimSubsystem, Constants.zeroLaunchAngle),
        new MoveElevatorToPosCommand(m_elevatorSubsystem, Constants.zeroElevator),
        new MoveIntakeToPosCommand(m_intakeSubsystem, Constants.intakeUp));
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
    
    Center_5_Note_Middle_Note = new PathPlannerAuto("Center_5_Note_Middle_Note");
    Center_5_Note_Top_Note = new PathPlannerAuto("Center_5_Note_Top_Note");

    Center_4_Note_All_Close = new PathPlannerAuto("Center_4_Note_All_Close");
    Center_4_Note_All_Far = new PathPlannerAuto("Center_4_Note_All_Far");
    Center_4_Note_Close_Amp_Side_Far_Middle = new PathPlannerAuto("Center_4_Note_Close_Amp_Side_Far_Middle");
    Center_4_Note_Close_Amp_Side_Far_Straight_Back = new PathPlannerAuto("Center_4_Note_Close_Amp_Side_Far_Straight_Back");
    Center_4_Note_Close_Amp_Side_Far_Top = new PathPlannerAuto("Center_4_Note_Close_Amp_Side_Far_Top");
    Center_4_Note_Close_Source_Side_Far_Middle = new PathPlannerAuto("Center_4_Note_Close_Source_Side_Far_Middle");
    Center_4_Note_Close_Source_Side_Far_Straight_Back = new PathPlannerAuto("Center_4_Note_Close_Source_Side_Far_Straight_Back");
    Center_4_Note_Close_Source_Side_Far_Top = new PathPlannerAuto("Center_4_Note_Close_Source_Side_Far_Top");
  
    Amp_Side_3_Note = new PathPlannerAuto("Amp_Side_3_Note");
    Center_3_Note_With_Middle = new PathPlannerAuto("Center_3_Note_With_Middle");
    Center_3_Note_With_Striaght_Back_Note = new PathPlannerAuto("Center_3_Note_With_Striaght_Back_Note");
    Center_3_Note_With_Top_Note = new PathPlannerAuto("Center_3_Note_With_Top_Note");
    Source_Side_3_Note = new PathPlannerAuto("Source_Side_3_Note");

    Amp_Side_To_Amp_Side_Note = new PathPlannerAuto("Amp_Side_To_Amp_Side_Note");
    Center_To_Amp_Side_Note = new PathPlannerAuto("Center_To_Amp_Side_Note");
    Center_To_Center_Note = new PathPlannerAuto("Center_To_Center_Note");
    Center_To_Source_Side_Note = new PathPlannerAuto("Center_To_Source_Side_Note");
    Source_Side_To_Source_Side_Note = new PathPlannerAuto("Source_Side_To_Source_Side_Note");

    Full_Disrupt_From_Amp_Side = new PathPlannerAuto("Full_Disrupt_From_Amp_Side");
    Full_Disrupt_From_Source_Side = new PathPlannerAuto("Full_Disrupt_From_Source_Side");

    m_chooser = new SendableChooser<>();
    m_chooser.addOption("Center_5_Note_Middle_Note", Center_5_Note_Middle_Note);
    m_chooser.addOption("Center_5_Note_Top_Note", Center_5_Note_Top_Note);

    m_chooser.addOption("Center_4_Note_All_Close", Center_4_Note_All_Close);
    m_chooser.addOption("Center_4_Note_All_Far", Center_4_Note_All_Far);
    m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Middle", Center_4_Note_Close_Amp_Side_Far_Middle);
    m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Straight_Back", Center_4_Note_Close_Amp_Side_Far_Straight_Back);
    m_chooser.addOption("Center_4_Note_Close_Amp_Side_Far_Top", Center_4_Note_Close_Amp_Side_Far_Top);
    m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Middle", Center_4_Note_Close_Source_Side_Far_Middle);
    m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Straight_Back", Center_4_Note_Close_Source_Side_Far_Straight_Back);
    m_chooser.addOption("Center_4_Note_Close_Source_Side_Far_Top", Center_4_Note_Close_Source_Side_Far_Top);

    m_chooser.addOption("Amp_Side_3_Note", Amp_Side_3_Note);
    m_chooser.addOption("Center_3_Note_With_Middle", Center_3_Note_With_Middle);
    m_chooser.addOption("Center_3_Note_With_Striaght_Back_Note", Center_3_Note_With_Striaght_Back_Note);
    m_chooser.addOption("Center_3_Note_With_Top_Note", Center_3_Note_With_Top_Note);
    m_chooser.addOption("Source_Side_3_Note", Source_Side_3_Note);
    
    m_chooser.addOption("Amp_Side_To_Amp_Side_Note", Amp_Side_To_Amp_Side_Note);
    m_chooser.addOption("Center_To_Amp_Side_Note", Center_To_Amp_Side_Note);
    m_chooser.addOption("Center_To_Center_Note", Center_To_Center_Note);
    m_chooser.addOption("Center_To_Source_Side_Note", Center_To_Source_Side_Note);
    m_chooser.addOption("Source_Side_To_Source_Side_Note", Source_Side_To_Source_Side_Note);

    m_chooser.addOption("Full_Disrupt_From_Amp_Side", Full_Disrupt_From_Amp_Side);
    m_chooser.addOption("Full_Disrupt_From_Source_Side", Full_Disrupt_From_Source_Side);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
