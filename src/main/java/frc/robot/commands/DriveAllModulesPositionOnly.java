// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveAllModulesPositionOnly extends CommandBase {
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
    if ((Math.abs(Robot.robotContainer.getDriverAxis(Axis.LEFT_Y)) > .1) || (Math.abs(Robot.robotContainer.getDriverAxis(Axis.LEFT_X)) > .1)){
      rotatePos = Math.atan2(Robot.robotContainer.getDriverAxis(Axis.LEFT_Y),Robot.robotContainer.getDriverAxis(Axis.LEFT_X));
    }
    for (int i=0 ; i<4 ; i++){
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
