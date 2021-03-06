// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * This command is for turning the PIDF loop on the modules' 
 * rotation motors, and other such test. It rotates the 
 * modules to a direction based on the movement of the 
 * joystick. Move the joystick in a direction to make all 
 * modules should go to that direction.
 */
public class DriveAllModulesPositionOnly extends CommandBase {
  //create a value for the modules to rotate to, we will update this later
  private double rotatePos = 0;
  /** Creates a new DriveAllModulesPositionOnly. */
  public DriveAllModulesPositionOnly() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check to see that the joystick is far enough from centered
    if ((Math.abs(Robot.robotContainer.getDriverAxis(Axis.LEFT_Y)) > .1)
        || (Math.abs(Robot.robotContainer.getDriverAxis(Axis.LEFT_X)) > .1)){
      //take the angle the joystick is moved to and make that the rotation target position
      rotatePos = Math.atan2(Robot.robotContainer.getDriverAxis(Axis.LEFT_Y),
        Robot.robotContainer.getDriverAxis(Axis.LEFT_X));
    }
    for (int i=0 ; i<4 ; i++){
      //assign the rotational direction to each module
      RobotContainer.swerveDrive.driveOneModule(i, 0, rotatePos, kDriveMode.percentOutput);
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
