// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GetSmol;
import frc.robot.commands.Drive.autoFunc.DrivePathWeaverProfile;
import frc.robot.commands.Drive.autoFunc.DriveStraightenAllModules;
import frc.robot.commands.Drive.autoFunc.DriveTurnToAngle;
import frc.robot.commands.Drive.autoFunc.DriveTurnToTarget;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.Shooter.ProgTBallWithHintOfLime;
import frc.robot.commands.Shooter.ResetBallsShot;
import frc.robot.commands.SnekLoader.Load;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootThenSwitchThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoShootThenSwitchThenShoot. */
  public AutoShootThenSwitchThenShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetBallsShot(),
      parallel(
        new PrepHoodShot(),//start the wheel, get the hood up
        sequence(
          // new DriveTurnToAngle(.25).withTimeout(.5),//if we need this the value will be small
          new DriveTurnToTarget().withTimeout(.5)//rotate to the target
        )
      ),
      new ProgTBallWithHintOfLime(0.0,false).withTimeout(1.5),//shoot with the hood up, till we're out of balls
      new GetSmol(),
      new DriveTurnToAngle(0).withTimeout(1.5),//turn back to the starting position
      new DriveStraightenAllModules().withTimeout(.5),
      new DrivePathWeaverProfile("SwitchGrabpt1"),
      // race(
      //   new Load(),
        sequence(
          new DriveTurnToAngle(-1.1783),
          new DrivePathWeaverProfile("SwitchGrabpt2"),
          new WaitCommand(.75),//wait a moment to get the balls that might be stray
          new DrivePathWeaverProfile("SwitchGrabpt3"),
          new WaitCommand(.75)
        )
      // )
      ,
      new DrivePathWeaverProfile("SwitchGrabpt4"),
      parallel(
        new PrepHoodShot(),//start the wheel, get the hood up
        sequence(
          new DriveTurnToAngle(1.1783).withTimeout(1.5),//if we need this the value will be small
          new DriveTurnToTarget().withTimeout(.5)//rotate to the target
        )
      ),
      new ProgTBallWithHintOfLime(0.0,false).withTimeout(1.5),//shoot with the hood up, till we're out of balls
      new GetSmol()
    );
  }
}
