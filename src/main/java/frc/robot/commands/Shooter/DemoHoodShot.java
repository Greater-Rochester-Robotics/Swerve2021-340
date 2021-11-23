// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class DemoHoodShot extends CommandBase {
  private Timer initTimer = new Timer();
  private Timer ballTimer = new Timer();
  private double timeBetweenBalls = .5;
  private int shootingBall = 1;
  
  /** 
   * This command shoots all the balls, but 
   * starts with one ball and then starts moving 
   * each progessive ball forward delayed by 0.5 
   * seconds after that previous ball.
   * this is done at Demo Speed.
   * 
   * @requires SnekLoader, Shooter 
   */
  public DemoHoodShot(){
    this(.5);
  }

  /** 
   * This command shoots all the balls, but 
   * starts with one ball and then starts moving 
   * each progessive ball forward delayed by 
   * timeBetweenBalls seconds after that previous ball. 
   * this is done at Demo Speed.
   * 
   * @param timeBetweenBalls time between when oone ball starts moving and the next
   * @requires SnekLoader, Shooter 
   */
  public DemoHoodShot(double timeBetweenBalls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeBetweenBalls = timeBetweenBalls;
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setShooterWheel(Constants.DEMO_SHOT_RPM);
    initTimer.reset();
    initTimer.start();
    shootingBall = 1;//in this command balls are counted up from the shooter
    RobotContainer.shooter.raiseHood();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter.isShooterAtSpeed()){
      RobotContainer.snekLoader.setPause(false);
      if(shootingBall == 1){
        RobotContainer.snekLoader.setState(State.kShootBall4);
      }else if(shootingBall == 2){
        RobotContainer.snekLoader.setState(State.kShootBall3);
      }else if(shootingBall == 3){
        RobotContainer.snekLoader.setState(State.kShootBall2);
      }else if(shootingBall == 4){
        RobotContainer.snekLoader.setState(State.kShootBall1);
      }else{
        RobotContainer.snekLoader.setState(State.kShootBall0);
      }
      
      if(ballTimer.hasElapsed(timeBetweenBalls)){
        //if time since the last ball was send to the shooter is big enough, allow next
        shootingBall++;
        //and then reset the timer, so we can judge for the next ball in sequence
        ballTimer.reset();
        ballTimer.start();
      }

    } else{
      RobotContainer.snekLoader.setPause(true);
      ballTimer.reset();
      ballTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.snekLoader.setState(State.kOff);
    RobotContainer.shooter.stop();
    RobotContainer.snekLoader.setPause(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
