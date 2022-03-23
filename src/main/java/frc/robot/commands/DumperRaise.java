// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;
import frc.robot.Constants;


public class DumperRaise extends CommandBase {
  /** Creates a new DumperMoveUp. */
  Dumper m_dumper;
  private boolean finish;

  public DumperRaise(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
    addRequirements(m_dumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dumper.setToCoast();
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_dumper.getTopSwitch().get()) {
      m_dumper.setToBrake();
      m_dumper.isUp();
      finish = true;
    }
    else {
      m_dumper.moveArm(Constants.dumperUpSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumper.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
