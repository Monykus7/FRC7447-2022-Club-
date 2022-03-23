// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnTimed extends CommandBase {
  /** Creates a new TurnTimed. */
  DriveTrain m_driveTrain;
  private boolean finish;
  double turnSpeed;
  double time;
  Timer timer;
  public TurnTimed(DriveTrain dt, double ds, double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    turnSpeed = ds;
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
    System.out.println("TurnTimed command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.turnTimed(turnSpeed);
    System.out.println("Driving forward");
    if (timer.get() >= time) {
      System.out.println("Time setpoint reached");
      timer.stop();
      finish  = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
