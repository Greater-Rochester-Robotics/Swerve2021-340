// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * This is a test command for driving the positional 
 * control PID loops for the translation, i.e this is 
 * used to tune the PID for position pid.
 */

public class DriveToPosition extends CommandBase {
  private Pose2d target;
  private Rotation2d currAngle;
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
    currAngle = RobotContainer.swerveDrive.getCurrentPose().getRotation();
  }
  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //TODO:Call the driveFieldCentric method with following paramters
    //TODO:call the position PIDController for away using param of the Pose2d x(getX()) value for awaySpeed
    double awaySpeed = RobotContainer.swerveDrive.awayPosPidController.calculate(RobotContainer.swerveDrive.getCurrentPose().getX(), target.getX());
    //TODO:call the position PIDController for Lateral using param of Pose2d y(getY()) for lateralSpeed
    double latSpeed = RobotContainer.swerveDrive.lateralPosPidController.calculate(RobotContainer.swerveDrive.getCurrentPose().getY(), target.getY());
    //TODO:call the position PIDController for rotation using param of Pose2d angle(.getRotation().getRadians()), for rotationSpeed
    //double rotSpeed = RobotContainer.swerveDrive.robotSpinController.
    //TODO: above with DutyCycle mode
    //RobotContainer.swerveDrive.driveFieldCentric(awaySpeed, latSpeed, rotSpeed, DutyCycle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:Stop all the motors
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//TODO: compare currentPosition() to target, use Math.abs() on each difference in X, Y and Angle
  }
}
