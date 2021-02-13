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
    //TODO:call the generateArcAngles() method and store the array
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //somhow we need to get the distance traveled, and I dont want to reset the Pose, but might have to
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:call the driveArc() function, using the generated arcArray and speed, also use dutyCycle mode
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
