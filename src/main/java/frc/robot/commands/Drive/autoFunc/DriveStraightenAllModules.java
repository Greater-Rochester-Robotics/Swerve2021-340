// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveStraightenAllModules extends CommandBase {
  boolean isFinished;
  /** 
   * This is a command used to make all the modules point 
   * foward or backward so as to make the path driving easier.
   */
  public DriveStraightenAllModules() {
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
    //first get all the current angles of the modules
    double[] currentAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    //default the finished boolean to true
    isFinished = true; 
    // itterate trough all the modules
    for(int i=0 ; i<3 ; i++){
      //tell this module to turn to 0.0
      RobotContainer.swerveDrive.driveOneModule(i, 0.0, 0.0, kDriveMode.percentOutput);
      //run a rolling boolean check of each module, within -2 to 2 degrees, or outside -178 or 178
      isFinished = isFinished && 
        (((currentAngles[i] > -2) && (currentAngles[i] < 2)) || (currentAngles[i] < -178) || (currentAngles[i] > 178) );
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the modules when done
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
