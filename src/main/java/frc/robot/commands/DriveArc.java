/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveArc extends CommandBase {
  private double driveSpd;
  private double arcCenterAng; // This is the angle relative to the front of the robot.
  private double arcRad;
  private double arcDist;
  private boolean tidLock;
  private double[][] moduleVectors; // Speed and direction for all modules.

  // Tidally locked means that as we drive around a point, our orientation relative to that point stays the same.
  // Non tidally locked means that as we drive around a point, our orientation relative to the field stays the same.
  // For now, we will only do tidally locked.
  public DriveArc(double driveSpeed,double arcCenterAngle,double arcRadius, boolean tidalLock) {
    
    this(driveSpeed, arcCenterAngle, arcRadius, 100, tidalLock);
  }
  
  public DriveArc(double driveSpeed,double arcCenterAngle,double arcRadius,double arcDistance, boolean tidalLock) {
    driveSpd = driveSpeed;
    arcCenterAng = arcCenterAngle;
    arcRad = arcRadius; 
    arcDist = arcDistance;
    tidLock = tidalLock;
    moduleVectors = new double[2][4];

    addRequirements(RobotContainer.swerveDrive);
    moduleVectors = RobotContainer.swerveDrive.generateArcAngles(arcRadius, arcCenterAngle); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //somehow we need to get the distance traveled, and I dont want to reset the Pose, but might have to
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turning along the arc.
    RobotContainer.swerveDrive.driveArc(driveSpd, moduleVectors, kDriveMode.percentOutput);
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
