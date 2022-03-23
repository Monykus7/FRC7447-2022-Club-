// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;
import frc.robot.Constants;

public class DumperIntake extends CommandBase {
  /** Creates a new MoveDumper. */
  Dumper m_dumper;

  public DumperIntake(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
    addRequirements(m_dumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dumper.checkArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dumper.intakeOuttake(Constants.dumperPower);

    if (m_dumper.getArmPosition() == -1){
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
