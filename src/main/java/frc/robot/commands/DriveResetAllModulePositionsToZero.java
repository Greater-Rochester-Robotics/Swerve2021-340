/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class DriveResetAllModulePositionsToZero extends InstantCommand {
  /**
   * Creates a new DriveResetAllModulePositionsToZero.
   */
  public DriveResetAllModulePositionsToZero() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Call the reset function to zero all of the modules 
    RobotContainer.swerveDrive.resetAllModules();
  }

}
