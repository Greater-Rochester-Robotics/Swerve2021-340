/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class FullSendsWall extends CommandBase {
  /**
   * Creates a new FullSendsWall.
   */
  private boolean fullSend;
  private int ballsToShoot;
  public FullSendsWall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballsToShoot = RobotContainer.snekLoader.getBallsLoaded();
    fullSend = false;
    RobotContainer.shooter.setShooterWheel(Constants.WALL_SHOT_RPM+200);
    RobotContainer.shooter.resetBallsShot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fullSend){
      RobotContainer.snekLoader.setPause(false);
      RobotContainer.snekLoader.setState(State.kShootBall0);
    } else{
      fullSend = (RobotContainer.shooter.isShooterAtSpeed());
      RobotContainer.snekLoader.setPause(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
    RobotContainer.snekLoader.setPause(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((RobotContainer.shooter.getBallsShot() >= ballsToShoot) && ballsToShoot > 0);
  }
}
