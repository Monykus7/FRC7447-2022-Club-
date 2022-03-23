// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDeez;
import frc.robot.Constants;

public class HellaMasculineLED extends CommandBase {
  /** Creates a new HellaMasculineLED. */
  LEDeez m_led;

  public HellaMasculineLED(LEDeez l) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = l;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.PurpleBlueWhite();
    Timer.delay(0.2f);
    m_led.PurpleBlueWhite2();
    Timer.delay(0.2f);
    m_led.PurpleBlueWhite3();
    Timer.delay(0.2f);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.White();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
