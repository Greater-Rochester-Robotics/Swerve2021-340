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
      new DriveStraightTrapProfile(Math.toRadians(0), 3.5052, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.424, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 3.85, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.424, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 3.148, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(0), 3.048, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(90), 1.524, 0, 0),
      new DriveStraightTrapProfile(Math.toRadians(180), 7.62, 0, 0)
    );
  }
}
