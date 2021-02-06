/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAdjustModuleZeroPoint extends CommandBase {
  /**
   * Creates a new DriveAdjustModuleZeroPoint.
   */
  public DriveAdjustModuleZeroPoint() {
    //TODO:addRequirements use addRequirements() and pull the subSystem object from RobotContainer
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:read the position of all modules and print them
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:Call the reset function to zero all of the modules 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//TODO: Creat a condition to end this command, so when that condition is met, the modules are zerooed
  }
}
