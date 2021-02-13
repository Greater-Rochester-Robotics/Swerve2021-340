// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * this command is for testing PID values for the drive
 */
public class DriveStraightAtSpeed extends CommandBase {
  /** Creates a new DriveStraightAtSpeed. */
  public DriveStraightAtSpeed(double speed, double angle, kDriveMode mode) {
    //TODO: Use addRequirements() here to declare subsystem dependencies.
    //todo: pass inputs to field values
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: call driveStraight passing in values from the constructor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:call stopAllMotors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
