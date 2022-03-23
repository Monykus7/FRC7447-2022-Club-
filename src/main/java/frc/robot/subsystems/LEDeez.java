// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDeez extends SubsystemBase {
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  /** Creates a new LEDeez. */
  public LEDeez() {
    m_led = new AddressableLED(Constants.RGBPort);
    m_ledBuffer = new AddressableLEDBuffer(Constants.RGBLength);
    m_led.setLength(m_ledBuffer.getLength());
  }

  public  void teamColors() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setRGB(i, 66, 277, 245);
      }
      else {
        m_ledBuffer.setRGB(i, 232, 253, 255);
      }
    }
  }

  

  public void rainbow() {
    int m_rainbowFirstPixelHue = 1;
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void purpleOne() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 3 == 0) {
        m_ledBuffer.setRGB(i, 106, 0, 255);
      } 
      else if (i % 3 == 1) {
        m_ledBuffer.setRGB(i, 178, 75, 242);
      }
      else {
        m_ledBuffer.setRGB(i, 180, 180, 180);
      }
    }
    m_led.setData(m_ledBuffer);
  }
  
  public void purple() {
    purpleOne();
    m_led.setData(m_ledBuffer);
  }


  public void blue() {
    blueOne();
    m_led.setData(m_ledBuffer);
  }

  public void blueOne() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 3 == 0) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      } 
      else if (i % 3 == 1) {
        m_ledBuffer.setRGB(i, 5, 133, 255);;
      }
      else {
        m_ledBuffer.setRGB(i, 180, 180, 180);
      }
    }
    m_led.setData(m_ledBuffer);
  }
  
  public void WhiteOne() {
    for (int i =0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 255);
    }
    m_led.setData(m_ledBuffer);

  }

  public void PurpleBlueWhite() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 3 == 0) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      } 
      else if (i % 3 == 1) {
        m_ledBuffer.setRGB(i, 178, 75, 242);
      }
      else {
        m_ledBuffer.setRGB(i, 200, 200, 200);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void PurpleBlueWhite2() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 3 + 1 == 0) {
        m_ledBuffer.setRGB(i, 200, 200, 200);
      } 
      else if (i%3 == 1) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
      else if (i%3 == 2) {
        m_ledBuffer.setRGB(i, 178, 75, 242);
      }
    }
    m_led.setData(m_ledBuffer);

  }

  public void PurpleBlueWhite3() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 3 == 0) {
        m_ledBuffer.setRGB(i, 178, 75, 242);
      } 
      else if (i % 3 == 1) {
        m_ledBuffer.setRGB(i, 200, 200, 200);
      }
      else if (i%3 ==2) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void White() {
    WhiteOne();
    m_led.setData(m_ledBuffer);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
