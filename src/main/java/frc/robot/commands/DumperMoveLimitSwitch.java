// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Dumper;

public class DumperMoveLimitSwitch extends CommandBase {
  private Dumper m_dumper;
  private boolean finish;
  private Timer m_timer;

  /** Creates a new DumperMoveLimitSwitch. */
  public DumperMoveLimitSwitch(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
    addRequirements(m_dumper);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    m_dumper.checkArmPosition();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If arm is down, move up
    if (m_dumper.getArmPosition() == -1) {
      if (m_dumper.getTopSwitch().get()) {
        m_dumper.isUp();
        m_dumper.setToBrake();
        finish = true;
      }
      else {
        m_dumper.setToCoast();
        m_dumper.moveArm(Constants.dumperUpSpeed);
        System.out.println("Moving upppp");
      }
  }

  // If arm is up, move down
    else if(m_dumper.getArmPosition() == 1) {
      if (m_dumper.getBottomSwitch().get()) {
        m_dumper.isDown();
        m_dumper.setToBrake();
        finish = true;
      }
      else {
        m_dumper.setToCoast();
        m_dumper.moveArm(Constants.dumperDownSpeed);
        System.out.println("moving downnnnnn!!");
      }
    }

    // If arm is middle, move up
    else if (m_dumper.getArmPosition() == 0) {
      if (m_dumper.getTopSwitch().get()) {
        m_dumper.isUp();
        m_dumper.setToBrake();
        finish = true;
      }
      else {
        m_dumper.setToCoast();
        m_dumper.moveArm(Constants.dumperUpSpeed);
        System.out.println("Moving upppp");
      }
    }

    if (m_timer.get() >= 2.0) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumper.stopArm();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
