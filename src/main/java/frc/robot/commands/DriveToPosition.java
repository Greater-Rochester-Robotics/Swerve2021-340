// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This is a test command for driving the positional 
 * control PID loops for the translation, i.e this is 
 * used to tune the PID for position pid.
 */
public class DriveToPosition extends CommandBase {
  //TODO:Create a field Pose2d to store target 
  
  /**
   * Constructs a new DriveToPostion. it takes a Pose2d 
   * object which is made of distance in x, distance in 
   * y, and a rotational position. The starting point of 
   * the robot is assumed to be 0=x and 0=y
   * @param target the pose the robot should have at the end of the command
   */
  public DriveToPosition(Pose2d target) {
    //TODO:Use addRequirements() to add drive as requirement
    //TODO:Pass parameter to field variable
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:reset the current position of the drivetrain to zero
    //TODO:get the current angle, and store in field variable for angle
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:Call the driveFieldCentric method with following paramters
    //TODO:call the position PIDController for away using param of the Pose2d x(getX()) value for awaySpeed
    //TODO:call the position PIDController for Lateral using param of Pose2d y(getY()) for lateralSpeed
    //TODO:call the position PIDController for rotation using param of Pose2d angle(.getRotation().getRadians()), for rotationSpeed
    //TODO: above with DutyCycle mode 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:Stop all the motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//TODO: compare currentPosition() to target, use Math.abs() on each difference in X, Y and Angle
  }
}
