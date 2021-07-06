/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveArcDriverControl extends CommandBase {
  private double driveSpd; //The speed we will drive on the arc
  private double arcCenterAng; // This is the angle relative to the front of the robot.
  private double arcRad; //the radius of the arc we are driving
  private double[][] moduleVectors = new double[4][2]; // Speed reduction factor and direction for all modules.
  private boolean isFinished = false;//a finished boolean, when inpputs are too small
  private double targetAngle;//The amount of turn the robot will go through
  private double targetGyroAngle;//the angle at the end of turning

  /**
   * A drive command that, when started, will take the current direction
   * the robot is being told to move, and drive a circular arc starting 
   * in the same direction, with an arcRadius to the immediate right or 
   * left of the starting point. The robot will make a total turn of the 
   * input targetAngle, or until the command is canceled.
   * 
   * @param arcRadius the radius of the arc the robot is going to drive
   * @param targetAngle the amount of turn the robot will move
   */
  public DriveArcDriverControl (double arcRadius, double targetAngle){
    this.arcRad = arcRadius;
    this.targetAngle = targetAngle * Math.signum(arcRadius);
    addRequirements(RobotContainer.swerveDrive);
  }

  /**
   * A drive command that, when started, will take the current direction
   * the robot is being told to move, and drive a circular arc starting 
   * in the same direction, with an arcRadius to the immediate right or 
   * left of the starting point. The robot will make a 90 degree turn.
   * 
   * @param arcRadius the radius of the arc the robot is going to drive
   */
  public DriveArcDriverControl (double arcRadius){
    this(arcRadius, Constants.PI_OVER_TWO);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //First, check the direction the robot is being told to go
    double  awaySpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_X))>.1){
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y)*.5;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)*.5;
    }
    awaySpeed*=-Constants.DRIVER_SPEED_SCALE_LATERAL;
    lateralSpeed*=-Constants.DRIVER_SPEED_SCALE_LATERAL;

    //Find out the total output speed the robot is being told to go
    driveSpd = Math.sqrt((awaySpeed*awaySpeed) + (lateralSpeed *lateralSpeed));
    //If drive speed is too small, end this command
    if (driveSpd < .01){
      isFinished=true;
      return;
    }

    //pull the current orientation of the robot as a Rotation2d object
    Rotation2d gyro = RobotContainer.swerveDrive.getGyroRotation2d();
    //find the forward and the strafe speed of the robot
    double robotForwardSpeed = (gyro.getCos()*awaySpeed) + (gyro.getSin() * lateralSpeed);
    double robotStrafeSpeed = (gyro.getCos()*lateralSpeed) - (gyro.getSin() * awaySpeed);

    //if arc radius is positive we are going counter clockwise, negaative is clockwise
    if (arcRad > 0){ 
      arcCenterAng = Math.atan2(robotStrafeSpeed, robotForwardSpeed) + Constants.PI_OVER_TWO;
      targetGyroAngle = RobotContainer.swerveDrive.getGyroInRad() + targetAngle;
      moduleVectors = RobotContainer.swerveDrive.generateArcAngles(arcRad, arcCenterAng); 
      driveSpd *= -1;
    }
    else {
      arcCenterAng = Math.atan2(robotStrafeSpeed, robotForwardSpeed) - Constants.PI_OVER_TWO;
      targetGyroAngle = RobotContainer.swerveDrive.getGyroInRad() + targetAngle;
      moduleVectors = RobotContainer.swerveDrive.generateArcAngles(-arcRad, arcCenterAng); 
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turning along the arc.
    if (!isFinished){
      RobotContainer.swerveDrive.driveArc(driveSpd, moduleVectors, kDriveMode.percentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished || Math.abs(RobotContainer.swerveDrive.getGyroInRad() - targetGyroAngle) < .03;
  }
}
