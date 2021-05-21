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
public class AutoTrench2BallShoot3TrenchShoot extends SequentialCommandGroup {
  /** Creates a new AutoTrench2BallShoot3TrenchShoot. */
  public AutoTrench2BallShoot3TrenchShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new Load(),//start loading
        sequence(
          new DrivePathWeaverProfile("TrenchRunpt1"),//drive back to pick up first ball
          new DrivePathWeaverProfile("TrenchRunpt2"),
          new WaitCommand(2)
        )
      ),
      new GetSmol(),
      new ParallelCommandGroup(
        new PrepHoodShot(),
        new DriveTurnToTarget()
      ),
      new FastBallWithHintOfLime().withTimeout(2),
      new GetSmol(),
      new DriveTurnToAngle(0.0),
      new ParallelRaceGroup(
        new Load(),
        sequence(
          new DrivePathWeaverProfile("TrenchRunpt2"),
          new DrivePathWeaverProfile("TrenchRunpt3"),
          new WaitCommand(2)
        )
      ),
      new GetSmol(),
      new DrivePathWeaverProfile("TrenchRunpt6"),
      new ParallelCommandGroup(
        new PrepHoodShot(),
        new DriveTurnToTarget()
      ),
      new FastBallWithHintOfLime().withTimeout(2),
      new GetSmol()
    );
  }
}
