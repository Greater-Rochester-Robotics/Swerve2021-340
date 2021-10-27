// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.Shooter.FastBallWithHintOfLime;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.Shooter.ProgTBallWithHintOfLime;
import frc.robot.commands.SnekLoader.Load;
import frc.robot.commands.*;
import frc.robot.commands.Drive.autoFunc.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoColorWheelStealThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoGetTwoGenBallsThenShoot. */
  public AutoColorWheelStealThenShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new Load(),//start loading
        sequence(
          //new DriveTurnToAngle(.2618),
          deadline(
            new WaitCommand(1.5),//wait for the harvest,, because slow now
            new DriveStraightenAllModules()
          ),
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
      new DriveTurnToAngle(-0.4).withTimeout(2.0),
      new DriveTurnToTarget().withTimeout(.5),
      new ProgTBallWithHintOfLime(.1).withTimeout(7.5),
      new GetSmol()
    );
  }
}
