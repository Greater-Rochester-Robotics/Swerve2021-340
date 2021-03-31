// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBouncePath2 extends SequentialCommandGroup {
  /** Creates a new AutoBouncePath2. */
  public AutoBouncePath2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPosition(new Pose2d(1.3,0,new Rotation2d())),
      new DriveToPosition(new Pose2d(0,1.1,new Rotation2d())),
      new DriveToPosition(new Pose2d(0,-1.1,new Rotation2d()))
    );
  }
}
