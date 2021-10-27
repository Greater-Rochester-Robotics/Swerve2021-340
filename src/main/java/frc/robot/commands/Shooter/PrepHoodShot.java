/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader;

public class PrepHoodShot extends CommandBase {
  Timer timer = new Timer();
  /**
   * Creates a new PrepHoodShot. This starts the 
   * shooter wheel spinning at an intial starting 
   * (warmup speed) speed and sets the hood to the 
   * proper position(up).
   * 
   * @requires Shooter
   */
  public PrepHoodShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: Update the below constant to use a new value of 10000 for warmup. Recommend creating new constant value
    RobotContainer.shooter.setShooterWheel(Constants.INITIATION_SHOT_RPM);
    RobotContainer.shooter.raiseHood();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.setShooterPrepped(timer.hasElapsed(1.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.shooter.isShooterAtSpeed());
  }
}
