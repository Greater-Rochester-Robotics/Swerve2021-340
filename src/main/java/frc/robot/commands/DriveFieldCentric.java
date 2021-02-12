/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;

public class DriveFieldCentric extends CommandBase {
  /**
   * Creates a new DriveFieldCentric.
   */
  public DriveFieldCentric() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.swerveDrive.driveFieldCentric(
      Robot.robotContainer.getDriverAxis(Axis.LEFT_Y) ,
      Robot.robotContainer.getDriverAxis(Axis.LEFT_X) ,
      Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}