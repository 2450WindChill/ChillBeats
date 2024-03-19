// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakeSubsystem extends SubsystemBase {
     // TODO: Change motor ids
     public final CANSparkFlex intakeMotor = new CANSparkFlex(13, MotorType.kBrushless);
     // public final CANSparkMax angleIntakeMotor = new CANSparkMax(27, MotorType.kBrushless);
     // public final SparkPIDController angleIntakeController = angleIntakeMotor.getPIDController();


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // CHANGE P AND I VALUES LATER RAAAAAH
    // angleIntakeController.setP(.01);
    // angleIntakeController.setOutputRange(-0.6, 0.6);
    // angleIntakeController.setI(.000003);
    // angleIntakeMotor.setIdleMode(Constants.angleBrakeMode);
  }

  /**
   * Example command factory method
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void intakeOn() {
    System.out.println("intake turn on");
    intakeMotor.set(0.1);
  }

  public void intakeOff() {
    System.out.println("intake turn off");
    intakeMotor.set(0);
  }
}
