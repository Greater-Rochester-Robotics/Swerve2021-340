// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

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
 * does not end of its own accord so it must be interrupted 
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
    double  awaySpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_X))>.1){
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y)*.5;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)*.5;
    }
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_TRIGGER) - Robot.robotContainer.getDriverAxis(Axis.LEFT_TRIGGER);

    //test if the absolute rotational input is greater than .1
    //if the test is true, just copy the DriveFieldCentric execute method
    //if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
    if (Math.abs(rotSpeed) > .1){
      RobotContainer.swerveDrive.driveFieldCentric(
        awaySpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        rotSpeed*-Constants.DRIVER_ROTATIONAL_SCALE ,
        kDriveMode.percentOutput
      );
      currentAngle = RobotContainer.swerveDrive.getGyroInRad();
    }
    else {
      RobotContainer.swerveDrive.driveFieldCentric(
        awaySpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
        RobotContainer.swerveDrive.getRobotRotationPIDOut(currentAngle),
        kDriveMode.percentOutput
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
