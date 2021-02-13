// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveStraightTrapProfile extends CommandBase {
  private TrapezoidProfile profile;
  private Timer timer = new Timer();
  private Pose2d initialPose;

  /** Creates a new DriveStraightTrapProfile. */
  public DriveStraightTrapProfile(double angle,TrapezoidProfile.State target,TrapezoidProfile.State start) {
    //TODO: Use addRequirements() here to declare subsystem dependencies.
    //TODO:Pass angle to fields
    profile = new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0),
            // Goal state
            target,
            // Initial state
            start);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPose = RobotContainer.swerveDrive.getCurrentPose();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:use profile to create a speed to the motors
    //TODO:use driveStraight method to pass values to the motor, should be in velocity mode
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(profile.totalTime());
  }

  public double distanceBetweenPose(Pose2d a,Pose2d b){
    return Math.sqrt( Math.pow(a.getX()+b.getX(),2) + Math.pow(a.getY()+b.getY(),2) );
  }
}
