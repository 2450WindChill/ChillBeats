// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.MoveElevatorToPosCommand

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AnkleCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FullIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.MoveElevatorToPosCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightySubsystem;
import frc.robot.subsystems.IndexSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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

  public Command autoLaunch() {
    return (Commands.runOnce(() -> m_ShootSubsystem.turnOnLauncher(), m_ShootSubsystem)
    .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOnIndexer(), m_IndexSubsystem))
    .andThen(new WaitCommand(2))
    .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOffIndexer(), m_IndexSubsystem))
    .andThen(Commands.runOnce(() -> m_ShootSubsystem.turnOffLauncher(), m_ShootSubsystem)));
    }

  public void setLEDsToAlliance() {
    teamColor = DriverStation.getAlliance().get();
    if (teamColor == DriverStation.Alliance.Red) {
      System.err.println("Alliance RED");
      m_LightySubsystem.SetLEDsToRed();
    } else {
      System.err.println("Alliance BLUE");
      m_LightySubsystem.SetLEDsToBlue();
    }
  }

  public void rainbow() {
    m_LightySubsystem.rainbow();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
