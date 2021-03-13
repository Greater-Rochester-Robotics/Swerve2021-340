/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * This command sets the current position of the modules as the zero
 * degrees. All the modules should be facing forward when running 
 * this command. this command can be run while the robot is disabled.
 * As a safety 
 */
public class DriveResetAllModulePositionsToZero extends CommandBase {
  Timer timer = new Timer();
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
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if(!interrupted){
      RobotContainer.swerveDrive.zeroAllModulePosSensors();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= 10;
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
