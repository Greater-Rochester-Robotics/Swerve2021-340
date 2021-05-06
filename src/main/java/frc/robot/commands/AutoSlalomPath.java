// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Running the slalom path using DriveStraight and DriveArc commands
 */
public class AutoSlalomPath extends SequentialCommandGroup {
  /** Creates a new AutoSalomPath. */
  public AutoSlalomPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 4.772, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.624, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 1.624, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 1.774, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 4.772, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.574, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 1.524, 0, 0)
    );
  }
}
