// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * this command is for testing PID values for the drive
 */
public class DriveStraightAtSpeed extends CommandBase {
  /** Creates a new DriveStraightAtSpeed. */
  private double speed;
  private double angle;
  private kDriveMode mode;
  
  public DriveStraightAtSpeed(double speed, double angle, kDriveMode mode) {
    addRequirements(RobotContainer.swerveDrive);

    this.speed = speed;
    this.angle = angle;
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calls driveStraight passing in values from the constructor
    RobotContainer.swerveDrive.driveStraight(speed, angle, mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
      RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
