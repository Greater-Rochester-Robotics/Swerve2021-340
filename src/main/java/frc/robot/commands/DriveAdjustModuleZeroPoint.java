/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kSwerveModule;

public class DriveAdjustModuleZeroPoint extends CommandBase {
  /**
   * Creates a new DriveAdjustModuleZeroPoint.
   * 
   * Each module has a different 0 angle. To solve this, physically angle each module straight. 
   * Then, tell all modules its current position is at 0. 
   */ 
  public DriveAdjustModuleZeroPoint() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Call the stop all command onto all the modules so they can freely spin.
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read the position of all modules and print them
    double[] modAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    //TODO: test smartdashboard outputs
    SmartDashboard.putNumber("frontLeft", modAngles[kSwerveModule.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRight", modAngles[kSwerveModule.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeft", modAngles[kSwerveModule.rearLeft.getNumber()]);
    SmartDashboard.putNumber(kSwerveModule.rearRight.toString(), modAngles[kSwerveModule.rearRight.getNumber()]);
    SmartDashboard.putNumber("gyroAngle",RobotContainer.swerveDrive.imu.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriveResetAllModulePositionsToZero should handle this 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
