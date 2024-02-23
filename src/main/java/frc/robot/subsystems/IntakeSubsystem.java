// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public final CANSparkMax intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
  public final CANSparkFlex ankleMotor = new CANSparkFlex(13, MotorType.kBrushless);
  public final SparkPIDController ankleController = ankleMotor.getPIDController();

  public final DigitalInput notelimitSwitch = new DigitalInput(0);
  public final DigitalInput toplimitSwitch = new DigitalInput(1);
  public final DigitalInput bottomlimitSwitch = new DigitalInput(2);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    ankleController.setP(.01);
    ankleController.setOutputRange(-0.2, 0.2);
    ankleController.setI(.00001);
    ankleMotor.setIdleMode(Constants.angleBrakeMode);
    ankleMotor.setClosedLoopRampRate(1.0);
    ankleMotor.setSmartCurrentLimit(40);
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
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("PewPew", notelimitSwitch.get());
    SmartDashboard.putNumber("Kankle", ankleMotor.getEncoder().getPosition());
   SmartDashboard.putNumber("intakeangle", intakeMotor.getEncoder().getPosition());
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

 
}
