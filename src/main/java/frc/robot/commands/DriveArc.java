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
  private double driveSpd; //The speed we will drive on the arc
  private double angleToArcCenterAbs;//The angle from the robot to the circle center, measured from field x-axis
  private double arcCenterAng; // This is the angle relative to the front of the robot.
  private double arcRad; //the radius of the arc we are driving
  private double[][] moduleVectors = new double[4][2]; // Speed reduction factor and direction for all modules.
  private double targetAngle;//The amount of turn the robot will go through
  private double targetGyroAngle;//the angle at the end of turning

  /**
   * 
   * @param driveSpeed the speed we will drive on the arc (-1.0 to  1.0)
   * @param angleToArcCenterAbs the absolute angle of to the arc center from field relative(radians)
   * @param arcRadius the radius of the arc the robot is going to drive
   * @param targetAngle the amount of turn the robot will move
   */
  public DriveArc(double driveSpeed, double angleToArcCenterAbs, double arcRadius, double targetAngle) {
    addRequirements(RobotContainer.swerveDrive);
    driveSpd = driveSpeed;
    arcRad = arcRadius;
    this.angleToArcCenterAbs = angleToArcCenterAbs;
    this.targetAngle = targetAngle;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arcCenterAng = 0.0;
    moduleVectors = RobotContainer.swerveDrive.generateArcAngles(arcRad, arcCenterAng); 
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
    return  Math.abs(RobotContainer.swerveDrive.getGyroInRad() - targetGyroAngle) < .03;
  }
}
