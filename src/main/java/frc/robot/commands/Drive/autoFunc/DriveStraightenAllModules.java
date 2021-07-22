// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveStraightenAllModules extends CommandBase {
  boolean isFinished;
  /** Creates a new DriveStraightenAllModules. */
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
    double[] currentAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    isFinished = true; 
    for(int i=0 ; i<3 ; i++){
      RobotContainer.swerveDrive.driveOneModule(i, 0.0, 0.0, kDriveMode.percentOutput);
      isFinished = isFinished && 
        (((currentAngles[i] > -2) && (currentAngles[i] < 2)) || ((currentAngles[i] < -178) || (currentAngles[i] > 178)));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
