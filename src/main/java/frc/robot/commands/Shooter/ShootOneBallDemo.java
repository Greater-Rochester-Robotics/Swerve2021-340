// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class ShootOneBallDemo extends CommandBase {
  private boolean fullSend;
  private Timer initTimer = new Timer();
  private State fireState = State.kOff;
  private boolean withHood;

  public ShootOneBallDemo(){
    this(false);
  }

  /** Creates a new ShootOneBall. */
  public ShootOneBallDemo(boolean withHood) {
    this.withHood = withHood;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fullSend = false;

    if(withHood){
      RobotContainer.shooter.raiseHood();
    }
    
    if(RobotContainer.snekLoader.getHandleSensor(4)){
      fireState = State.kShootBall4;
    }else if(RobotContainer.snekLoader.getHandleSensor(3)){
      fireState = State.kShootBall3;
    }else if(RobotContainer.snekLoader.getHandleSensor(2)){
      fireState = State.kShootBall2;
    }else if(RobotContainer.snekLoader.getHandleSensor(1)){
      fireState = State.kShootBall1;
    }else if(RobotContainer.snekLoader.getHandleSensor(0)){
      fireState = State.kShootBall0;
    }else{
      fireState = State.kOff;
    }

    RobotContainer.shooter.setShooterWheel(Constants.DEMO_SHOT_RPM);
    initTimer.reset();
    initTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fullSend){
      RobotContainer.snekLoader.setPause(false);
      RobotContainer.snekLoader.setState(fireState);
    } else{
      fullSend = (RobotContainer.shooter.isShooterAtSpeed());
      RobotContainer.snekLoader.setPause(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.snekLoader.setState(State.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
