// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * This is a test command for driving the positional 
 * control PID loops for the translation, i.e this is 
 * used to tune the PID for position pid.
 */

public class DriveToPosition extends CommandBase {
  private Pose2d target;
  
  /**
   * Constructs a new DriveToPosition. it takes a Pose2d 
   * object which is made of distance in x, distance in 
   * y, and a rotational position. The starting point of 
   * the robot is assumed to be 0=x and 0=y
   * @param target the pose the robot should have at the end of the command
   */
    public DriveToPosition(Pose2d target) {
    addRequirements(RobotContainer.swerveDrive);
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetCurrentPos();
  }
  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //Call the driveFieldCentric method with following parameters
    //call the position PIDController for away using param of the Pose2d x(getX()) value for awaySpeed
    double awaySpeed = RobotContainer.swerveDrive.getAwayPositionPIDOut(target.getX());
    //call the position PIDController for Lateral using param of Pose2d y(getY()) for lateralSpeed
    double latSpeed = RobotContainer.swerveDrive.getLateralPositionPIDOut(target.getY());
    //call the position PIDController for rotation using param of Pose2d angle(.getRotation().getRadians()), for rotationSpeed
    double rotSpeed = RobotContainer.swerveDrive.getRobotRotationPIDOut(target.getRotation().getRadians());
    //above with DutyCycle mode
    RobotContainer.swerveDrive.driveFieldCentric(awaySpeed*.5, latSpeed*.5, rotSpeed*.5, kDriveMode.percentOutput);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop all the motors
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  //compare currentPosition() to target, use Math.abs() on each difference in X, Y and Angle
  @Override
  public boolean isFinished() {
    Pose2d curPos = RobotContainer.swerveDrive.getCurrentPose();
    if( Math.abs(target.getX()-curPos.getX()) < 0.05
      && Math.abs(target.getY()-curPos.getY()) < 0.05
      && Math.abs(target.getRotation().getRadians()-curPos.getRotation().getRadians()) < 0.01
    ){
      return true;
    }
    else {
      return false;
    }
  }
}
