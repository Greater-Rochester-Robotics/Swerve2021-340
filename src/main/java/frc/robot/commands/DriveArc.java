/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveArc extends CommandBase {
  private double driveSpd;
  private double arcCenterAng;
  private double arcRad;
  private double arcDist;
  private boolean tidLock;

  public DriveArc(double driveSpeed,double arcCenterAngle,double arcRadius, boolean tidalLock) {
    
    this(driveSpeed, arcCenterAngle, arcRadius, 100, tidalLock);
  }
  
  public DriveArc(double driveSpeed,double arcCenterAngle,double arcRadius,double arcDistance, boolean tidalLock) {
    driveSpd = driveSpeed;
    arcCenterAng = arcCenterAngle;
    arcRad = arcRadius; 
    arcDist = arcDistance;
    tidLock = tidalLock;

    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
