// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightySubsystem extends SubsystemBase {
 
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  DriverStation.Alliance m_teamColor;
  private int m_rainbowFirstPixelValue = 0;

  private int m_snakeindex = 0;
  
  
  /** Creates a new LightySubsystem. */
  public LightySubsystem(DriverStation.Alliance teamColor) { 
   m_rainbowFirstPixelValue = 0;
    m_teamColor = teamColor;

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
     m_led = new AddressableLED(9);

  
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(75);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    SetLEDsToBlue();
    
  }


  public void SetLEDsToBlue(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 50, 155);
   }
   m_led.setData(m_ledBuffer);
  }
  
  public void SetLEDsToYellow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 155, 75, 0);
   }
   m_led.setData(m_ledBuffer);
  }

  public void SetLEDsToPurple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 155,0 , 155);
   }
   m_led.setData(m_ledBuffer);
  }
  public void SetLEDsToGreen(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0,155 , 0);
   }
   m_led.setData(m_ledBuffer);
  }

  public void SetLEDsToRed(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 155,0 , 0);
   }
   
   m_led.setData(m_ledBuffer);

  } 
  
  public void rainbow() {

    var T = Timer.getFPGATimestamp();

    var amplitude = .5;
    var frequency = .05;

    var value = (amplitude*Math.sin(frequency*T)+.5);
    var red = (int) (0*value*255);
    var green = (int) (0*value*255);
    var blue = (int) (155*value*255);
    
      // System.err.println("Red"+red); 
      // System.err.println("Green"+green);
      // System.err.println("Blue"+blue);

    // For every pixel
    m_rainbowFirstPixelValue += 3;
    // Check bounds
    m_rainbowFirstPixelValue %= 100;

    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //   m_ledBuffer.setHSV(i, 100, 255, m_rainbowFirstPixelValue);
    // }
    // Increase by to make the rainbow "move"
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 100, 255, 0);
    }  
    m_ledBuffer.setHSV(m_snakeindex, 100, 255, 100);
    m_snakeindex += 1;
    m_snakeindex %= 75;
    m_led.setData(m_ledBuffer);
      
  }
    }

// public void setLEDsToAlliance() {
  // teamColor = DriverStation.getAlliance().get();
  // if (teamColor == DriverStation.Alliance.Red) {
  // System.err.println("Alliance RED");
  // m_LightySubsystem.SetLEDsToRed();
  // } else {
  // System.err.println("Alliance BLUE");
  // m_LightySubsystem.SetLEDsToBlue();
  // }

  // public void rainbow() {
  // m_LightySubsystem.rainbow();
  // }

  // public Command autoLaunch() {
  // return (new MoveWristToPoseCommand(m_AimSubsystem, Constants.launchAngle))
  // .HoldWristCommand()
  // .alongWith(Commands.runOnce(() -> m_ShootSubsystem.turnOnLauncher(),
  // m_ShootSubsystem))
  // .andThen(new WaitCommand(3))
  // .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOnIndexer(),
  // m_IndexSubsystem))
  // .andThen(new WaitCommand(2))
  // .stophold
  // .andThen(Commands.runOnce(() -> m_IndexSubsystem.turnOffIndexer(),
  // m_IndexSubsystem))
  // .andThen(Commands.runOnce(() -> m_ShootSubsystem.turnOffLauncher(),
  // m_ShootSubsystem));
  // }
