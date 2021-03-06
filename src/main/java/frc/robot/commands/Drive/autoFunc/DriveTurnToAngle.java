// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveTurnToAngle extends CommandBase {
  private double angle = 0;

  /** Creates a new DriveTurnToAngle. This command 
   * is used in tuning PID for rotation and in 
   * autonomous to make a specific turn. 
   * 
   * @param angle angle in radians 
   */
  public DriveTurnToAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);

    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, RobotContainer.swerveDrive.getRobotRotationPIDOut(angle), kDriveMode.percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - RobotContainer.swerveDrive.getGyroInRad()) < .03;
  }
}
