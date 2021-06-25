// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SnekLoader.State;

public class ProgBallWithHintOfLime extends CommandBase {
  private int speedRpm;
  private Timer initTimer = new Timer();
  private Timer ballTimer = new Timer();

  private boolean prevShootSensor = false;
  private int shootingBall = 1;
  
  /** Creates a new ProgBallWithHintOfLime. */
  public ProgBallWithHintOfLime() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.raiseHood();
    RobotContainer.limelight.setLightState(3);

    // if(RobotContainer.limelight.getDistance() > 96 && RobotContainer.limelight.getDistance() < 120){
    //   speedRpm = 19000;
    // }
    // else{
    //   speedRpm = Limelight.calcHoodRPM();
    // }


    // if(RobotContainer.limelight.getDistance() > 96 && RobotContainer.limelight.getDistance() < 120){
      speedRpm = 17500;
    // }
    // else{
    //   speedRpm = Limelight.calcHoodRPM();
    // }

    
    RobotContainer.shooter.setShooterWheel(speedRpm);
    initTimer.reset();
    initTimer.start();
    shootingBall = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter.isShooterAtSpeed() && initTimer.hasElapsed(1.5)){
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
      
      if((RobotContainer.shooter.getShooterSensor() && !prevShootSensor) || ballTimer.hasElapsed(.5)){
        shootingBall++;
        ballTimer.reset();
        ballTimer.start();
      }
      prevShootSensor = RobotContainer.shooter.getShooterSensor();

    } else{
      RobotContainer.snekLoader.setPause(true);
      ballTimer.reset();
      ballTimer.start();
    }

    SmartDashboard.putNumber("shootingBall",shootingBall);
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
