// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    public final CANSparkMax indexMotor = new CANSparkMax(20, MotorType.kBrushless);
    public final DigitalInput indexBeamBreak = new DigitalInput(1);

  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    indexMotor.setSmartCurrentLimit(40);
  }

  /**
   * Example command factory method.
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

  // public void turnOnIndexer() {
  //   System.out.println("turn on");
  //   indexMotor.set(-1);
  // }

  // public void turnOffIndexer() {
  //     System.out.println("turn off");
  //   indexMotor.set(0);
  // }

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
        //  SmartDashboard.putNumber("rubikx",indexMotor.getEncoder().getPosition());
         SmartDashboard.putBoolean("Index Beam Break", indexBeamBreak.get());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void indexOn() {
    System.out.println("index turn on");
    indexMotor.set(-0.7536932);
  }

  public void indexReverse() {
    indexMotor.set(0.7);
  }

  public void indexOff() {
    System.out.println("index turn off");
    indexMotor.set(0);
  }

}
