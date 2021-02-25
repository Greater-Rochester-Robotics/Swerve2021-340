// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * for use with https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html
 */
public class DriveTuneDriveMotorFeedForward extends CommandBase {
  //TODO:Create a field variable to store the speed
  /** Creates a new DriveTuneDriveMotorFeedForward. */
  public DriveTuneDriveMotorFeedForward() {
    //TODO:assign constructor speed argument to field value
    //TODO: Use addRequirements() here to use subsystem.
    //TODO:Print all module velocities
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:use driveOneModule for each motor, position setting m0 to 135, m1 to -135, m2 to -45, and m3 to 45, set speed to field speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:run the stop motors method
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
