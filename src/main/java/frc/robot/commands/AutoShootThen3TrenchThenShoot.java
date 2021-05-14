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
import frc.robot.commands.Shooter.ResetBallsShot;
import frc.robot.commands.SnekLoader.Load;
import frc.robot.commands.DriveTurnToTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootThen3TrenchThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoShootThenOurTrenchThenShoot. */
  public AutoShootThen3TrenchThenShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new ResetBallsShot(),
      new ParallelCommandGroup(
        new PrepHoodShot(),//start the wheel, get the hood up
        sequence(
          new DriveTurnToAngle(.25),
          new DriveTurnToTarget()//rotate to the target
        )
      ),
      new FastBallWithHintOfLime().withTimeout(2.5),//shoot with the hood up, till we're out of balls
      new GetSmol(),// shrink back to fit under the color wheel thing
      new DriveTurnToAngle(0),//turn back to the starting position
      new ParallelRaceGroup(
        new Load(),//start loading
        sequence(
          new DrivePathWeaverProfile("TrenchRunpt1"),//drive back to pick up first ball
          new DrivePathWeaverProfile("TrenchRunpt2"),//pick up second ball
          new DrivePathWeaverProfile("TrenchRunpt2"),//pick up third ball
          new WaitCommand(1)//wait a moment to get the balls that might be stray
        )
      ),
      new GetSmol(),
      new DrivePathWeaverProfile("TrenchRunpt5")// drive forward enough to shoot
    //   new PrepHoodShot(),// once we're clear of color wheel, start the shooter, hood up
    //   new DriveTurnToTarget(),//target the goal
    //   new FastBallWithHintOfLime().withTimeout(2.5)
    );
  }
}
