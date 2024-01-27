// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
    public final CANSparkFlex intakeMotor = new CANSparkFlex(13, MotorType.kBrushless);
    public final CANSparkFlex ankleMotor = new CANSparkFlex(14, MotorType.kBrushless);

    public final DigitalInput notelimitSwitch = new DigitalInput(0);
    public final DigitalInput toplimitSwitch = new DigitalInput(1);
    public final DigitalInput bottomlimitSwitch = new DigitalInput(2);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

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
   SmartDashboard.putBoolean("PewPew", notelimitSwitch.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public void setAnkleSpeed(double speed) {
  //   if (speed > 0) {
  //     if (toplimitSwitch.get()) {
  //       //stop motor if top limit is tripped
  //       ankleMotor.set(0);
  //     } else {
  //       //move motor if top limit is not tripped
  //       ankleMotor.set(speed);
  //     }
  //   } else if (speed < 0) {
  //     if (bottomlimitSwitch.get()) {
  //       //stop motor if bottom limit is reached
  //       ankleMotor.set(0);
  //     } else {
  //       //move motor if bottom limit is not reached
  //       ankleMotor.set(speed);
  //     }
  //   } else {
  //     //stop motor if 0 speed in inputed
  //     ankleMotor.set(0);
  //   }
  // }
}
