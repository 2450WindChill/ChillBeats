// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightySubsystem extends SubsystemBase {
 
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  DriverStation.Alliance m_teamColor;
  
  
  /** Creates a new LightySubsystem. */
  public LightySubsystem(DriverStation.Alliance teamColor) { 
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

}
