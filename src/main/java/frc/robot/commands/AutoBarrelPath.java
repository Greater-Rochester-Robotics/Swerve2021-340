// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Running the barrel path using DriveStraight and DriveArc commands
 */
public class AutoBarrelPath extends SequentialCommandGroup {
  /** Creates a new AutoBarrelPath. */
  public AutoBarrelPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 1.2192, 0, 0),
    );
  }
}
