/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveFieldCentricArc extends CommandBase {
  private double driveSpd;
  private double arcCenterAng; // This is the angle relative to the front of the robot.
  private double arcRad;
  private double[][] moduleVectors = new double[4][2];; // Speed and direction for all modules.
  private boolean isFinished = false;
  private double targetGyroAngle;

  // Tidally locked means that as we drive around a point, our orientation relative to that point stays the same.
  // Non tidally locked means that as we drive around a point, our orientation relative to the field stays the same.
  // For now, we will only do tidally locked.
  public DriveFieldCentricArc (double arcRadius){
    this.arcRad = arcRadius;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double  awaySpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_X))>.1){
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y)*.5;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)*.5;
    }
    awaySpeed*=-Constants.DRIVER_SPEED_SCALE_LATERAL;
    lateralSpeed*=-Constants.DRIVER_SPEED_SCALE_LATERAL;

    driveSpd = Math.sqrt((awaySpeed*awaySpeed) + (lateralSpeed *lateralSpeed));
    if (driveSpd < .01){
      isFinished=true;
      return;
    }

    Rotation2d gyro = RobotContainer.swerveDrive.getGyroRotation2d();
    double robotForwardSpeed = (gyro.getCos()*awaySpeed) + (gyro.getSin() * lateralSpeed);
    double robotStrafeSpeed = (gyro.getCos()*lateralSpeed) - (gyro.getSin() * awaySpeed);

    if (arcRad > 0){ 
      arcCenterAng = Math.atan2(robotStrafeSpeed, robotForwardSpeed) + Constants.PI_OVER_TWO;
      targetGyroAngle = RobotContainer.swerveDrive.getGyroInRad() + Constants.PI_OVER_TWO;
      moduleVectors = RobotContainer.swerveDrive.generateArcAngles(arcRad, arcCenterAng); 
      driveSpd *= -1;
    }
    else {
      arcCenterAng = Math.atan2(robotStrafeSpeed, robotForwardSpeed) - Constants.PI_OVER_TWO;
      targetGyroAngle = RobotContainer.swerveDrive.getGyroInRad() - Constants.PI_OVER_TWO;
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
