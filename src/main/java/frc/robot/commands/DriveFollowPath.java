// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveFollowPath extends CommandBase {
  private Timer timer = new Timer();
  double Positon_P_Value = .55;
  double Positon_I_Value = 0;
  double Positon_D_Value = 0;

  PIDController awayPositionalController = 
    new PIDController(Positon_P_Value,Positon_I_Value,Positon_D_Value);
  PIDController lateralPositionalController = 
    new PIDController(Positon_P_Value,Positon_I_Value,Positon_D_Value);
  
  private Trajectory robotTrajectory = new Trajectory();;
  /** Creates a new DriveFollowPath. */
  public DriveFollowPath(String fileName) {
    addRequirements(RobotContainer.swerveDrive);

    String trajectoryJSON = "output/output/" + fileName + ".wpilib.json";
    
    try {
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      robotTrajectory = TrajectoryUtil.fromPathweaverJson(filePath);
    } 
    catch (IOException ex) {
      System.out.println("Unable to open trajectory: " + trajectoryJSON);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.swerveDrive.setCurrentPos(
      new Pose2d(robotTrajectory.getInitialPose().getTranslation(),
      RobotContainer.swerveDrive.getGyroRotation2d()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State targetState = robotTrajectory.sample(timer.get()+.02);
    Pose2d currentPose = RobotContainer.swerveDrive.getCurrentPose();
    double awaySpeed = awayPositionalController.calculate(
      currentPose.getX(), targetState.poseMeters.getX());
    double rotSpeed = 
      RobotContainer.swerveDrive.getRobotRotationPIDOut(
      targetState.poseMeters.getRotation().getRadians());

    double lateralSpeed = lateralPositionalController.calculate(
        currentPose.getY(), targetState.poseMeters.getY());
    RobotContainer.swerveDrive.driveFieldCentric(awaySpeed,
      lateralSpeed, rotSpeed, kDriveMode.percentOutput);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(robotTrajectory.getTotalTimeSeconds());
  }
}
