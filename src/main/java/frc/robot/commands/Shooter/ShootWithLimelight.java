/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;
import frc.robot.subsystems.Limelight;

public class ShootWithLimelight extends CommandBase {
  private int stateIndex;
  private int ballsToShoot;
  private int speedRpm;
  private Timer timer;
  private boolean wallShot;

  public ShootWithLimelight(boolean shootFromWall) {
    wallShot = shootFromWall;
    this.ballsToShoot = -1;
    System.out.println(
      "constructed"
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.snekLoader,RobotContainer.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.setLightState(3);
    if(RobotContainer.limelight.getDistance() < 96 || wallShot){
      RobotContainer.shooter.lowerHood();
    }
    else{
      RobotContainer.shooter.raiseHood();
    }
    if(wallShot){
      speedRpm = Constants.WALL_SHOT_RPM;
    }
    else{
      speedRpm = Limelight.calcHoodRPM();
    }
    RobotContainer.shooter.resetBallsShot();
    stateIndex = 4;
    RobotContainer.shooter.setShooterWheel(speedRpm);
    timer = new Timer();
    // System.out.println("Shoot init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //TODO: Determine if this is in the correct location to allow a more accurate speed. - Robot can move while aiming
    // speedRpm = Limelight.calcHoodRPM();
    // RobotContainer.shooter.setShooterWheel(speedRpm);
    // Check speed if PID loop isn't working for the flywheel to spin up between
    // shots
    // SmartDashboard.putString("TEST", "Happy");
    String shootSpeed = "";
    if(!timer.hasElapsed(0.1)){
      timer.start();
    }
    if (!timer.hasElapsed(2)){
      return;
    }
    else{
      if(!RobotContainer.shooter.isShooterAtSpeed()) {
        // SmartDashboard.putString("Speed?", "No");
        RobotContainer.snekLoader.setPause(true);
        // RobotContainer.snekLoader.setState(State.kFillTo4);
        // stateIndex=4;
        return;
        }
    }
    RobotContainer.snekLoader.setPause(false);
    // SmartDashboard.putString("Speed?", "Yes");
    if ((stateIndex == 4)) {
      RobotContainer.snekLoader.setState(State.kShootBall4);
      shootSpeed = RobotContainer.shooter.getShooterVelocity() + "";
      stateIndex = 3;
    } else if (stateIndex == 3 && (!RobotContainer.snekLoader.getHandleSensor(4))) {
      RobotContainer.snekLoader.setState(State.kShootBall3);
      shootSpeed = RobotContainer.shooter.getShooterVelocity() + "";
      stateIndex = 2;
    } else if (stateIndex == 2 && (!RobotContainer.snekLoader.getHandleSensor(3))) {
      RobotContainer.snekLoader.setState(State.kShootBall2);
      shootSpeed = RobotContainer.shooter.getShooterVelocity() + "";
      stateIndex = 1;
    } else if (stateIndex == 1 && (!RobotContainer.snekLoader.getHandleSensor(2))) {
      RobotContainer.snekLoader.setState(State.kShootBall1);
      shootSpeed = RobotContainer.shooter.getShooterVelocity() + "";
      stateIndex = 0;
    } else if (stateIndex == 0 && (!RobotContainer.snekLoader.getHandleSensor(1))) {
      RobotContainer.snekLoader.setState(State.kShootBall0);
      shootSpeed = RobotContainer.shooter.getShooterVelocity() + "";
      stateIndex = -1;
    }
    if(shootSpeed != "") {
      SmartDashboard.putString("Ball Shot At",shootSpeed);
    }
    // SmartDashboard.putString("index", ""+stateIndex);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    RobotContainer.limelight.setLightState(1);
    // System.out.println("Shoot() ended interrupted:" + interrupted);
    RobotContainer.shooter.stop();
    if ( interrupted){
    RobotContainer.snekLoader.setState(State.kOff);
    }else{
      RobotContainer.snekLoader.setState(State.kFillTo4);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((RobotContainer.shooter.getBallsShot() >= ballsToShoot) && ballsToShoot > 0);
  }
}
