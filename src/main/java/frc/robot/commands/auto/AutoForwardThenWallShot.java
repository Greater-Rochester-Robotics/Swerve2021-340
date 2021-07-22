// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GetSmol;
import frc.robot.commands.Shooter.FullWallShot;
import frc.robot.commands.Drive.autoFunc.DrivePathWeaverProfile;
import frc.robot.commands.Drive.autoFunc.DriveStraightenAllModules;
import frc.robot.commands.Shooter.PrepWallShot;
import frc.robot.commands.Shooter.ProgTWallShot;
import frc.robot.commands.Shooter.ResetBallsShot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoForwardThenWallShot extends SequentialCommandGroup {
  /** Creates a new AutoForwardThenWallShot. */
  public AutoForwardThenWallShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightenAllModules().withTimeout(1),
      new ResetBallsShot(),
      new DrivePathWeaverProfile("Forward"),
      new PrepWallShot(),
      new ProgTWallShot(.25).withTimeout(5),
      new GetSmol()
    );
  }
}
