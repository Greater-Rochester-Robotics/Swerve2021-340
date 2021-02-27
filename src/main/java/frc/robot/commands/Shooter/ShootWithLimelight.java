/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;
import frc.robot.subsystems.Limelight;

public class ShootWithLimelight extends CommandBase {
  private int stateIndex;
  private int ballsToShoot;
  private int speedRpm;

  public ShootWithLimelight() {
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
    RobotContainer.shooter.raiseHood();
    RobotContainer.limelight.setLightState(3);
    speedRpm = Limelight.calcHoodRPM();
    RobotContainer.shooter.resetBallsShot();
    stateIndex = 4;
    RobotContainer.shooter.setShooterWheel(speedRpm);
    // System.out.println("Shoot init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check speed if PID loop isn't working for the flywheel to spin up between
    // shots
    // SmartDashboard.putString("TEST", "Happy");
    if (!RobotContainer.shooter.isShooterAtSpeed()) {
      // SmartDashboard.putString("Speed?", "No");
      RobotContainer.snekLoader.setPause(true);
      // RobotContainer.snekLoader.setState(State.kFillTo4);
      // stateIndex=4;
      return;
    }
    RobotContainer.snekLoader.setPause(false);
    // SmartDashboard.putString("Speed?", "Yes");
    if ((stateIndex == 4)) {
      RobotContainer.snekLoader.setState(State.kShootBall4);
      stateIndex = 3;
    } else if (stateIndex == 3 && (!RobotContainer.snekLoader.getHandleSensor(4))) {
      RobotContainer.snekLoader.setState(State.kShootBall3);
      stateIndex = 2;
    } else if (stateIndex == 2 && (!RobotContainer.snekLoader.getHandleSensor(3))) {
      RobotContainer.snekLoader.setState(State.kShootBall2);
      stateIndex = 1;
    } else if (stateIndex == 1 && (!RobotContainer.snekLoader.getHandleSensor(2))) {
      RobotContainer.snekLoader.setState(State.kShootBall1);
      stateIndex = 0;
    } else if (stateIndex == 0 && (!RobotContainer.snekLoader.getHandleSensor(1))) {
      RobotContainer.snekLoader.setState(State.kShootBall0);
      stateIndex = -1;
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
