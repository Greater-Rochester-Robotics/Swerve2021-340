/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SnekLoader.State;

public class FastBallWithHintOfLime extends CommandBase {
  /**
   * Creates a new FastBallWithAddedLime.
   */
  private int speedRpm;
  private boolean fullSend;
  private int ballsToShoot;
  private Timer timer = new Timer();
  public FastBallWithHintOfLime() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader,RobotContainer.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.resetBallsShot();
    ballsToShoot = RobotContainer.snekLoader.getBallsLoaded();
    RobotContainer.shooter.raiseHood();
    RobotContainer.limelight.setLightState(3);
    if(RobotContainer.limelight.getDistance() > 96 && RobotContainer.limelight.getDistance() < 120){
      speedRpm = 17500;
    }
    else{
      speedRpm = Limelight.calcHoodRPM();
    }
    
    fullSend = false;
    RobotContainer.shooter.setShooterWheel(speedRpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(0.1)){
      timer.start();
    }
    if(fullSend){
      RobotContainer.snekLoader.setPause(false);
      RobotContainer.snekLoader.setState(State.kShootBall0);
    } else{
      //if(timer.hasElapsed(2.0)){
        fullSend = (RobotContainer.shooter.isShooterAtSpeed());
      RobotContainer.snekLoader.setPause(true);  
      //}
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
    return (((RobotContainer.shooter.getBallsShot() >= ballsToShoot) && ballsToShoot > 0));
  }
}