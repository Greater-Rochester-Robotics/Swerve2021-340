// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.FastBallWithHintOfLime;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.Shooter.ResetBallsShot;
import frc.robot.commands.SnekLoader.LoadAcc;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootThenOurTrenchThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoShootThenOurTrenchThenShoot. */
  public AutoShootThenOurTrenchThenShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new ResetBallsShot(),
      new PrepHoodShot(),
      new DriveTurnToTarget(),
      new FastBallWithHintOfLime().withTimeout(2.5),
      new DriveTurnToAngle(0),
      new Load()
    );
  }
}
