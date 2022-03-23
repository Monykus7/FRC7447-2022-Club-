// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Dumper;

public class DumperVomit extends CommandBase {
  Dumper m_dumper;

  /** Creates a new DumperVomit. */
  public DumperVomit(Dumper d) {
    m_dumper = d;
    addRequirements(m_dumper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dumper.checkArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dumper.intakeOuttake(Constants.dumperRetch);

    if (m_dumper.getArmPosition() == -1) {
      m_dumper.moveArm(Constants.dumperHoldDownSpeed);
    }
    else if (m_dumper.getArmPosition() == 1) {
      m_dumper.moveArm(Constants.dumperHoldUpSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumper.stopIntakeOuttake();
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
