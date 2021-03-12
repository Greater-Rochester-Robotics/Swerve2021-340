// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interupted 
 * to end.
 * 
 * UNLIKE DriveFieldCentric or DriveFieldCentricAdvanced 
 * this command uses three PIDControllers to maintain the 
 * robot's rotational orientation and position in each 
 * direction when the robot is not instructed to move in 
 * that way. 
 */

public class DriveFieldCentricAdvancedTwo extends CommandBase {
  private final double THRESHOLD = .1;
  private double currentAwayPos = 0;
  private double currentLateralPos = 0;
  private double currentAngle = 0;

  /** Creates a new DriveFieldCentricAdvancedTwo. */
  public DriveFieldCentricAdvancedTwo() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPosition = RobotContainer.swerveDrive.getCurrentPose();
    double currentAwayPos = RobotContainer.swerveDrive.getCurrentPose().getY();
    double currentLateralPos = RobotContainer.swerveDrive.getCurrentPose().getX();
    double currentAngle = RobotContainer.swerveDrive.getCurrentPose().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //create variables to hold output values
    double awayOutput = 0;
    double lateralOutput = 0;
    double rotOutput = 0;

    //pull current position of the robot
    Pose2d currentPosition = RobotContainer.swerveDrive.getCurrentPose();

    //pull joystick axis and place them into variables
    double awaySpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    double awaySpeedSlow = Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y);
    double lateralSpeedSlow = Robot.robotContainer.getDriverAxis(Axis.RIGHT_X);
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_TRIGGER) 
      - Robot.robotContainer.getDriverAxis(Axis.LEFT_TRIGGER);

    if(Math.abs(awaySpeedSlow) > THRESHOLD){
      //if slow stick greater than threshold, use that
      awayOutput = awaySpeedSlow*-Constants.DRIVER_SPEED_SCALE_LATERAL*.5;
      //update current position
      currentAwayPos = currentPosition.getX();
    }else if(Math.abs(awaySpeed) > THRESHOLD){
      //if fast stick greater than threshold, use that
      awayOutput = awaySpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL;
      //update current position
      currentAwayPos = currentPosition.getX();
    }else{
      //if no stick inputs, use the most recent position as a setpoint in a PID loop
      //set awayOutput to the awayPosPIDController, use currentAwayPos as param
      
    }

    if(Math.abs(lateralSpeedSlow) > THRESHOLD){
      //if slow stick greater than threshold, use that
      lateralOutput = lateralSpeedSlow*-Constants.DRIVER_SPEED_SCALE_LATERAL*.5;
      //update current position
      currentLateralPos = currentPosition.getY();
    }else if(Math.abs(lateralSpeed) > THRESHOLD){
      //if fast stick greater than threshold, use that
      lateralOutput = lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL;
      //update current position
      currentLateralPos = currentPosition.getY();
    }else{
      
    }

    if (Math.abs(rotSpeed) > THRESHOLD){
      rotOutput = rotSpeed*-Constants.DRIVER_ROTATIONAL_SCALE;
      currentAngle = currentPosition.getRotation().getRadians();  
    }else{
      rotOutput = RobotContainer.swerveDrive.getRobotRotationPIDOut(currentAngle);
    }

    //use outputs in the drivetrain
    RobotContainer.swerveDrive.driveFieldCentric(
      awayOutput,
      lateralOutput,
      rotOutput,
      kDriveMode.percentOutput
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
