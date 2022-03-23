// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class BangTurnToAngle extends CommandBase {
  DriveTrain m_driveTrain;
  double angleSetpoint;
  private boolean finish;

  /** Creates a new TurnToAngle. */
  public BangTurnToAngle(DriveTrain dt, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    angleSetpoint = a;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angleSetpoint > 0) {
      m_driveTrain.turn(Constants.autonTSpeed);
    }
    else {
      m_driveTrain.turn(-Constants.autonTSpeed);
    }
    

    if (m_driveTrain.getAngle() == angleSetpoint) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}