// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Running the bounce path using DriveStraight and DriveArc commands
 */
public class AutoBouncePath extends SequentialCommandGroup {
  /** Creates a new AutoBouncePath. */
  public AutoBouncePath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightTrapProfile(0, .46355, 0, 0),
      new DriveArc(1.9, Math.toRadians(90), .762, Math.toRadians(90)),
      new DriveStraightTrapProfile(Math.toRadians(90), 0.762, 1.9, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 0.762, 0, 1.9),
      new DriveArc(1.9, Math.toRadians(0), 0.762, Math.toRadians(30)),
      new DriveStraightTrapProfile(Math.toRadians(-60), 3, 1.9, 1.9), //distance needs to be corrected
      new DriveArc(1.9, Math.toRadians(-150), 0.762, Math.toRadians(-30)),
      new DriveArc(1.9, Math.toRadians(0), 0.762, Math.toRadians(180)),
      new DriveStraightTrapProfile(Math.toRadians(90), 2.286, 1.9, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 2.286, 0, 1.9),
      new DriveArc(1.9, Math.toRadians(0), 0.762, Math.toRadians(90)),
      new DriveStraightTrapProfile(Math.toRadians(0), 0.762, 1.9, 1.9),
      new DriveArc(1.9, Math.toRadians(90), 0.762, Math.toRadians(90)),
      new DriveStraightTrapProfile(Math.toRadians(90), 2.286, 1.9, 0),
      new DriveStraightTrapProfile(Math.toRadians(-90), 0.762, 0, 1.9),
      new DriveArc(1.9, Math.toRadians(0), 0.762, Math.toRadians(90))

    );
  }
}
