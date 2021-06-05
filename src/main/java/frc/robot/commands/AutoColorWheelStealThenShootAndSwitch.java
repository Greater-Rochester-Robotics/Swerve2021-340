// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.FastBallWithHintOfLime;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.SnekLoader.Load;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoColorWheelStealThenShootAndSwitch extends SequentialCommandGroup {
  /** Creates a new AutoGetTwoGenBallsThenShoot. */
  public AutoColorWheelStealThenShootAndSwitch() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new Load(),//start loading
        sequence(
          new DrivePathWeaverProfile("ColorWheelStealpt1"),//drive back to pick up balls
          // new DriveTurnToAngle(.17),
          // new DriveTurnToAngle(0),
          new WaitCommand(2)//wait a moment to get the balls that might be stray
        )
      ),
      new GetSmol(),
      new ParallelCommandGroup(
        new PrepHoodShot(),
        new DrivePathWeaverProfile("ColorWheelStealpt2")
      ),
      new DriveTurnToTarget().withTimeout(3),
      new FastBallWithHintOfLime().withTimeout(7.5),
      new GetSmol(),
      new DriveTurnToAngle(Math.toRadians(-77))//,
      // new DriveOathWeaverProfile("");

    );
  }
}
