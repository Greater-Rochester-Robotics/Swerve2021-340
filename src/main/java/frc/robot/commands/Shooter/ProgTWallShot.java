// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class ProgTWallShot extends CommandBase {
  private Timer initTimer = new Timer();
  private Timer ballTimer = new Timer();
  private double timeBetweenBalls = .5;
  private int shootingBall = 1;
  
  /** 
   * This command shoots all the balls, but 
   * starts with one ball and then starts moving 
   * each progessive ball forward delayed by 0.5 
   * seconds after that previous ball.
   * 
   * @requires SnekLoader, Shooter 
   */
  public ProgTWallShot(){
    this(.5);
  }

  /** 
   * This command shoots all the balls, but 
   * starts with one ball and then starts moving 
   * each progessive ball forward delayed by 
   * timeBetweenBalls seconds after that previous ball. 
   * 
   * @param timeBetweenBalls time between when oone ball starts moving and the next
   * @requires SnekLoader, Shooter 
   */
  public ProgTWallShot(double timeBetweenBalls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeBetweenBalls = timeBetweenBalls;
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setShooterWheel(Constants.WALL_SHOT_RPM);
    initTimer.reset();
    initTimer.start();
    shootingBall = 1;
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
        shootingBall++;
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
