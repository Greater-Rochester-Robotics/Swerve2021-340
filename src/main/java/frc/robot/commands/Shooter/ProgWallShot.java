// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class ProgWallShot extends CommandBase {
  private Timer timer = new Timer();
  private boolean prevShootSensor = false;
  private int shootingBall = 1;
  /** Creates a new ProgWallShot. */
  public ProgWallShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setShooterWheel(Constants.WALL_SHOT_RPM);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter.isShooterAtSpeed()){
      RobotContainer.snekLoader.setPause(false);
      RobotContainer.snekLoader.setPause(false);
      if(shootingBall == 5){
        RobotContainer.snekLoader.setState(State.kShootBall0);
      }else if(shootingBall == 4){
        RobotContainer.snekLoader.setState(State.kShootBall1);
      }else if(shootingBall == 3){
        RobotContainer.snekLoader.setState(State.kShootBall2);
      }else if(shootingBall == 2){
        RobotContainer.snekLoader.setState(State.kShootBall3);
      }else{
        RobotContainer.snekLoader.setState(State.kShootBall4);
      }
      
      if(RobotContainer.shooter.getShooterSensor() && !prevShootSensor){
        shootingBall++;
      }
      prevShootSensor = RobotContainer.shooter.getShooterSensor();

    } else{
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
    return false;
  }
}
