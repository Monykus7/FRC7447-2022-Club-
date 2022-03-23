// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTimed extends CommandBase {
  DriveTrain m_driveTrain;
  private boolean finish;
  double driveSpeed;
  double time;
  Timer timer;

  /** Creates a new DriveForwardTimed. */
  public DriveTimed(DriveTrain dt, double ds, double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    driveSpeed = ds;
    time = t;
    addRequirements(m_driveTrain);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    timer.reset();
    timer.start();
    System.out.println("DriveForwardTimed command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.driveTimed(driveSpeed);
    System.out.println("Driving forward");
    if (timer.get() >= time) {
      finish  = true;
      System.out.println("Time setpoint reached");
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
    System.out.println("Stopping motors and ending command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
