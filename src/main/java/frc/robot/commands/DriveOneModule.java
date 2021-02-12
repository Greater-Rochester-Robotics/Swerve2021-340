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
import frc.robot.subsystems.SwerveDrive;

/**
 * A command to test one module at a time, it takes that module as a constructor arguement
 */
public class DriveOneModule extends CommandBase {
  private int moduleNum;
  private int rotatePos = 0;
  /**
   * Creates a new DriveOneModule.
   */
  public DriveOneModule(int moduleNumber) {
    //use addRequirements() and pull the subSystem object from RobotContainer
    addRequirements(RobotContainer.swerveDrive);
    //assigns the passed module number to the field of similar name
    moduleNum = moduleNumber;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check to see if there is a new angle input from the DPad
    if(Robot.robotContainer.getDriverDPad() != -1){
      rotatePos = (int)Math.toRadians(Robot.robotContainer.getDriverDPad());
    }

    //Set the one module we are working with to an angle and a speed
    RobotContainer.swerveDrive.driveOneModule(moduleNum, 
      Robot.robotContainer.getDriverAxis(Axis.LEFT_Y), 
      rotatePos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
