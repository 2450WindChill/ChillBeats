// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveToPose;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final TestSubsystem m_TestSubsystem = new TestSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final AimSubsystem m_AimSubsystem = new AimSubsystem();
  private final LauncherSubsystem m_ShootSubsystem = new LauncherSubsystem();
  private final IndexSubsystem m_IndexSubsystem = new IndexSubsystem();
  public DriverStation.Alliance teamColor;
  private final LightySubsystem m_LightySubsystem = new LightySubsystem(teamColor);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController2 = new CommandXboxController(
      OperatorConstants.kDriverControllerPort2);
  

   static XboxController m_driverController = new XboxController(0);
  static XboxController m_operatorController = new XboxController(1);

  public final JoystickButton drive_rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  public final JoystickButton drive_yButton = new JoystickButton(m_driverController, Button.kY.value);
  public final JoystickButton buttonA = new JoystickButton(m_driverController, Button.kA.value);

  public Command path1;
  public Command path2;
  public Command path3;

  public Command moveForward;
  public SendableChooser<Command> m_chooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> false
          ));

    // Configure bindings and limelight
    configureBindings();
    configureAutoChooser();

    m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", m_chooser);

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
    private void configureBindings() {
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      // new Trigger(m_exampleSubsystem::exampleCondition)
      //     .onTrue(new ExampleCommand(m_exampleSubsystem));
  
      // Schedule `AnkleCommand` when the Xbox controller's X button is pressed,
      // cancelling on release.
      //m_driverController.x().onTrue(new MoveElevatorToPosCommand(m_ElevatorSubsystem, 20.0));
      // m_driverController.y().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_driverController));
      //m_driverController.a().onTrue(new MoveElevatorToPosCommand(m_ElevatorSubsystem, 0.0));
  
  
      m_driverController.b().onTrue(autoLaunch());
  
  
      // m_driverController.leftBumper().whileTrue(new IndexCommand(m_IndexSubsystem, m_driverController));
      //m_driverController.rightBumper().whileTrue(new LaunchCommand(m_ShootSubsystem, m_driverController));
  
  
    //  m_driverController2.rightBumper().whileTrue(new LaunchCommand(m_ShootSubsystem, m_driverController2, 0.8));
    //  m_driverController2.leftBumper().whileTrue(new LaunchCommand(m_ShootSubsystem, m_driverController2, 0.1));
      // m_driverController.x().onTrue(new FullIntakeCommand(m_IntakeSubsystem));
      //Laser laser
      m_driverController.rightBumper().onTrue(Commands.runOnce(() ->m_LightySubsystem.SetLEDsToRed()));
     
    }
    //drive_yButton.onTrue(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d()));
  }

  // public void configureAutoChooser() {
  //   moveForward = Commands.runOnce(() -> new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0.2, 0, new Rotation2d(0))));
  //   // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(-0.2, 0, new Rotation2d(0))))
  //   // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, -0.2, new Rotation2d(0))))
  //   // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, 0.2, new Rotation2d(0)))));

  //   m_chooser = new SendableChooser<>();
  //   m_chooser.setDefaultOption("Test Auto", moveForward);
  // }
                   

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

  //  public void setLEDsToAlliance() {
  //   teamColor = DriverStation.getAlliance().get();
  //   if (teamColor == DriverStation.Alliance.Red) {
  //     System.err.println("Alliance RED");
  //     m_LightySubsystem.SetLEDsToRed();
  //   } else {
  //     System.err.println("Alliance BLUE");
  //     m_LightySubsystem.SetLEDsToBlue();
  //   }

  // public void rainbow() {
  //   m_LightySubsystem.rainbow();
  // }

   public Command autoLaunch() {
    return (Commands.runOnce(() -> m_ShootSubsystem.turnOnLauncher(), m_ShootSubsystem)
    .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOnIndexer(), m_IndexSubsystem))
    .andThen(new WaitCommand(2))
    .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOffIndexer(), m_IndexSubsystem))
    .andThen(Commands.runOnce(() -> m_ShootSubsystem.turnOffLauncher(), m_ShootSubsystem)));
    }
   //.andThen(new MoveToPositionNoPID(m_ArmSubsystem, Constants.midRowPlacingAngle))
   public void configureAutoChooser() {
    Station_1_Shoot_Moveout = Commands.run(() -> ).andThen(new PathPlannerAuto("Station_1_Shoot_Moveout_Auto"));
    Station_2_Shoot_Moveout = Commands.run(() -> ).andThen(new PathPlannerAuto("Station_2_Shoot_Moveout_Auto"));
    Station_3_Shoot_Moveout = Commands.run(() -> ).andThen(new PathPlannerAuto("Station_3_Shoot_Moveout_Auto"));

    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("Station 1", Station_1_Shoot_Moveout);
    m_chooser.addOption("Station 2", Station_2_Shoot_Moveout);
    m_chooser.addOption("Station 3", Station_3_Shoot_Moveout);
   }


  public Command getAutonomousCommand() {
    System.out.println("AUTO IS RUNNING");
     // return
     // new TestAutoCommand(m_drivetrainSubsystem, m_poseEstimator, 5, 0);
     // Commands.run(() -> m_drivetrainSubsystem.testAuto(.4, 0), m_drivetrainSubsystem);
     // new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0.2, 0, new Rotation2d(0)));
    // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(-0.2, 0, new Rotation2d(0))))
    // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, -0.2, new Rotation2d(0))))
    // .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(0, 0.2, new Rotation2d(0))));
    return new PathPlannerAuto("SwerveAuto");
    //return autoChooser.getSelected();
}
    // // Example auto which moves the robot forward 5 meters and then back to the origin without rotating
    // return new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d(5, 0, new Rotation2d(0)))
    //           .andThen(new MoveToPose(m_drivetrainSubsystem, m_poseEstimator, new Pose2d()));
  }
