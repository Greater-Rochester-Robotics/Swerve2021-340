/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LimelightCommands.AutoAlign;
import frc.robot.commands.LimelightCommands.LimelightOff;
import frc.robot.commands.LimelightCommands.LimelightOn;
import frc.robot.commands.Shooter.Shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullInitShot extends SequentialCommandGroup {
  /**
   * Creates a new FullInitShot.
   */
  public FullInitShot() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new LimelightOn(),new PrepHoodShot(), new AutoAlign(), new Shoot(Constants.INITIATION_SHOT_RPM), new LimelightOff());
  }
}
