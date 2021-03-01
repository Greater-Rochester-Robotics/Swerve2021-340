/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.SwerveDrive.kSwerveModule;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * A command to test one module at a time, it takes that module 
 * as a constructor arguement. This command is designed to test 
 * a single module, while all other module are stopped. 
 */
public class DriveOneModule extends CommandBase {
  private int moduleNum;
  private double rotatePos = 0.0;
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
    //we are driving one module, stop all other modules so they don't keep running
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check to see if there is a new angle input from the DPad
    // System.out.println("Getting Driver pad" + Robot.robotContainer.getDriverDPad());
    if(Robot.robotContainer.getDriverDPad() != -1){
      rotatePos = Math.toRadians(Robot.robotContainer.getDriverDPad());
      rotatePos = (rotatePos - Math.PI);
    }

    double[] modAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    //TODO: test smartdashboard outputs
    SmartDashboard.putNumber("frontLeft", modAngles[kSwerveModule.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRight", modAngles[kSwerveModule.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeft", modAngles[kSwerveModule.rearLeft.getNumber()]);
    SmartDashboard.putNumber(kSwerveModule.rearRight.toString(), modAngles[kSwerveModule.rearRight.getNumber()]);

    // System.out.println("getting  kSwerveModule numbers!" + Robot.robotContainer.getDriverAxis(Axis.LEFT_Y) + " " + rotatePos);
    //Set the one module we are working with to an angle and a speed
    RobotContainer.swerveDrive.driveOneModule(
      moduleNum, Robot.robotContainer.getDriverAxis(Axis.LEFT_Y), rotatePos, kDriveMode.percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //before any other command is run, stop all modules, for safety 
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
