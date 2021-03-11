// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOnTargetWithLimeLight extends CommandBase {
  /** Creates a new DriveOnTargetWithLimeLight. */
  public DriveOnTargetWithLimeLight() {
    //TODO: Use addRequirements() here for just swervedrive
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:Turn On Limelight lights
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:copy exectute() loop from DriveFieldCentricAdvanced
    //TODO:remove references to rotSpeed and currentAngle, make the angle a value from the limelight
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO: turn off limelight lights
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
