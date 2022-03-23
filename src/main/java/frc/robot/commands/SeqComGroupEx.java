// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Dumper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeqComGroupEx extends SequentialCommandGroup {
  /** Creates a new AutonOne. */
  public SeqComGroupEx(DriveTrain dt, Dumper d) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveTimed(dt, 0.3, 3.0), new DumperVomit(d));
    //Dumper vomit wouldn't work here, since it's binded to a button, but put other auton commands in this format
    // - addCommands(new *insertcommandhere()*, new *insertcommandhere()*)
    // For any parameters, add it with the subsystem in the public area bit 
  }
}
