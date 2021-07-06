// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveOnTargetWithLimeLight extends CommandBase {
  /** Creates a new DriveOnTargetWithLimeLight. */
  public DriveOnTargetWithLimeLight() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.setLightState(3);
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
    
    double targetAngle = RobotContainer.swerveDrive.getGyroInRad();

    //
    if(RobotContainer.limelight.haveTarget())
    {
      targetAngle -= Math.toRadians(RobotContainer.limelight.angleToTarget());
    }
    // else{
    //   targetAngle /= (Math.PI);
    //   if(Math.floor(targetAngle)%2 != 0)
    //   {
    //     targetAngle = Math.floor(targetAngle);
    //   }else{
    //     targetAngle = Math.ceil(targetAngle);
    //   }
    //   targetAngle *= Math.PI;
    // }
    
    SmartDashboard.putNumber("angle to target", RobotContainer.limelight.angleToTarget());
    SmartDashboard.putNumber("target angle", targetAngle);
    SmartDashboard.putNumber("distance to target", RobotContainer.limelight.getDistance());
    RobotContainer.swerveDrive.driveFieldCentric(
      awaySpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
      lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LATERAL,
      RobotContainer.swerveDrive.getRobotRotationPIDOut(targetAngle) * 1.5,
      kDriveMode.percentOutput
    );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.limelight.setLightState(1);
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
