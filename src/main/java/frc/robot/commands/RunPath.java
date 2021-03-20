// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class RunPath extends CommandBase {
  private Timer timer = new Timer();
  private Trajectory robotPath;
  /** Creates a new RunPath. */
  public RunPath(String fileName) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);

    String trajectoryJSON = "output/output/" + fileName + ".wpilib.json";
    robotPath = new Trajectory();
    try {
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      robotPath = TrajectoryUtil.fromPathweaverJson(filePath);
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
    //Sets the initial position of the path to the current position of the robot
    //robotPath.transformBy(new Transform2d(robotPath.getInitialPose(), RobotContainer.swerveDrive.getCurrentPose()));
    RobotContainer.swerveDrive.setCurrentPos(new Pose2d(robotPath.getInitialPose().getTranslation(), RobotContainer.swerveDrive.getGyroRotation2d()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gives a current position between the points, based on the time in seconds
    State curState = robotPath.sample(timer.get());

    //TODO: DOUBLE CHECK ALL OF THIS
    Pose2d targetPose = curState.poseMeters;
    Rotation2d targetRotation = targetPose.getRotation();
    double targetVelocity = curState.velocityMetersPerSecond;
    double targetAngle = curState.curvatureRadPerMeter;
    double targetAcceleration = curState.accelerationMetersPerSecondSq;
    Pose2d currentVelocity = RobotContainer.swerveDrive.getCurrentVelocity();
    // 1. obtain the targetVelocity and the targetAngle from curState
    // 2. use the targetVelocity and targetAngle to determine away and lateral speed
    // 3. pass the away and lateral speeds to our PID loops (cereal)
    // 4. pass our targetAngle to the rotationPID
    // 5. pass our new speeds to driveFieldCentric
    double awaySpeedDiff = RobotContainer.swerveDrive.getAwaySpeedPIDFFOut(targetVelocity*targetRotation.getCos(), targetAcceleration*targetRotation.getCos());
    double latSpeedDiff = RobotContainer.swerveDrive.getLateralSpeedPIDFFOut(targetVelocity*targetRotation.getSin(), targetAcceleration*targetRotation.getSin());
    double rotSpeed = RobotContainer.swerveDrive.getRobotRotationPIDOut(targetAngle);

    SmartDashboard.putNumber("Away Speed", awaySpeedDiff + currentVelocity.getX());
    SmartDashboard.putNumber("Lateral Speed", latSpeedDiff + currentVelocity.getY());
    SmartDashboard.putNumber("Target Position X", targetPose.getX());
    SmartDashboard.putNumber("Target Position Y", targetPose.getY());

    RobotContainer.swerveDrive.driveFieldCentric(awaySpeedDiff + currentVelocity.getX(), latSpeedDiff + currentVelocity.getY(), rotSpeed, kDriveMode.percentOutput);

    try{
      Thread.sleep(100);
    } catch (InterruptedException e){
      e.printStackTrace();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotPath.getTotalTimeSeconds() <= timer.get();
  }
}
