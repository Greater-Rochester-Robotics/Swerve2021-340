// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;

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
 * UNLIKE DriveFieldCentric this command uses a PIDController 
 * to maintain the robot's rotational orientation when the 
 * robot is not instructed to rotate by the rotational 
 * input. 
 */

public class DriveFieldCentricAdvanced extends CommandBase {
  private double currentAngle = 0;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveFieldCentricAdvanced() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = Robot.robotContainer.swerveDrive.getGyroInRad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //test if the absolute rotational input is greater than .1
    //if the test is true, just copy the DriveFieldCentric execute method
    //if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
    if (Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)) > .1){
      RobotContainer.swerveDrive.driveFieldCentric(
        Robot.robotContainer.getDriverAxis(Axis.LEFT_Y)*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        Robot.robotContainer.getDriverAxis(Axis.LEFT_X)*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)*-Constants.DRIVER_ROTATIONAL_SCALE 
      );
      currentAngle = RobotContainer.swerveDrive.getGyroInRad();
    }
    else {
      RobotContainer.swerveDrive.driveFieldCentric(
        Robot.robotContainer.getDriverAxis(Axis.LEFT_Y)*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        Robot.robotContainer.getDriverAxis(Axis.LEFT_X)*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        Robot.robotContainer.swerveDrive.getRobotRotationPIDOut(currentAngle)
      );
    }
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
