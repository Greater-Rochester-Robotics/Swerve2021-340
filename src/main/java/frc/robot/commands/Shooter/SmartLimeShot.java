/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.LimelightCommands.AutoAlign;
import frc.robot.commands.LimelightCommands.LimelightOff;
import frc.robot.commands.LimelightCommands.LimelightOn;
import frc.robot.commands.LimelightCommands.ObtainDistance;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SmartLimeShot extends SequentialCommandGroup {
  /**
   * Creates a new LimeHoodShot.
   */
  public SmartLimeShot() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //Make a new limelight command dealing with all of the shooting and getting of distance
    super(new LimelightOn(), new AutoAlign() ,new FastBallWithHintOfLime(), new LimelightOff());
  }
}
