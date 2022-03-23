// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;

public class LimitSwitchTest extends CommandBase {
  Dumper m_dumper;

  /** Creates a new LimitSwitchTest. */
  public LimitSwitchTest(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Top switch hit: " + m_dumper.getTopSwitch().get());
    System.out.println("Bottom switch hit: " + m_dumper.getBottomSwitch().get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
