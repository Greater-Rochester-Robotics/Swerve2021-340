// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DrivePathWeaverProfile extends CommandBase {
  private Timer timer = new Timer();
  Trajectory trajectory = new Trajectory();
  private Translation2d initialPosition;
  private double targetAngle;
  private boolean angleTargeted;
  // private Translation2d currentDrivePosition;
  // private Translation2d currentDriveVelocity;
  
  PIDController awayPosPidController  = new PIDController(Constants.AWAY_POS_P, Constants.AWAY_POS_I, Constants.AWAY_POS_D);
  PIDController awaySpeedPIDController = new PIDController(Constants.AWAY_SPEED_P, Constants.AWAY_SPEED_I, Constants.AWAY_SPEED_D);
  SimpleMotorFeedforward awaySpeedFeedforward = new SimpleMotorFeedforward(Constants.AWAY_FEEDFORWARD_STATIC, Constants.AWAY_FEEDFORWARD_VELOCITY, Constants.AWAY_FEEDFORWARD_ACCELERATION);

  PIDController lateralPosPidController  = new PIDController(Constants.AWAY_POS_P, Constants.AWAY_POS_I, Constants.AWAY_POS_D);
  PIDController lateralSpeedPIDController = new PIDController(Constants.AWAY_SPEED_P, Constants.AWAY_SPEED_I, Constants.AWAY_SPEED_D);
  SimpleMotorFeedforward lateralSpeedFeedforward = new SimpleMotorFeedforward(Constants.AWAY_FEEDFORWARD_STATIC, Constants.AWAY_FEEDFORWARD_VELOCITY, Constants.AWAY_FEEDFORWARD_ACCELERATION);



  public DrivePathWeaverProfile(String fileName) {
    addRequirements(RobotContainer.swerveDrive);

    String trajectoryJSON = "output/"+ fileName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    angleTargeted=false;
    
  }

  public DrivePathWeaverProfile(String fileName, double targetAngle) {
    addRequirements(RobotContainer.swerveDrive);

    String trajectoryJSON = "output/"+ fileName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    angleTargeted = true;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setOdometryActive(true);
    if(!angleTargeted){
      targetAngle = RobotContainer.swerveDrive.getGyroInRad();
    }
    initialPosition = RobotContainer.swerveDrive.getCurrentPose().getTranslation();
    RobotContainer.swerveDrive.setCurrentPos(trajectory.getInitialPose());
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //use profile to create a position and speed for the motors
    Trajectory.State reference = trajectory.sample(timer.get());

    double targetPosX = reference.poseMeters.getX();
    double targetPosY = reference.poseMeters.getY();
    double targetVelX = reference.velocityMetersPerSecond*reference.poseMeters.getRotation().getCos();
    double targetVelY = reference.velocityMetersPerSecond*reference.poseMeters.getRotation().getSin();
    double targetAccX = reference.accelerationMetersPerSecondSq*reference.poseMeters.getRotation().getCos();
    double targetAccY = reference.accelerationMetersPerSecondSq*reference.poseMeters.getRotation().getSin();
    
    
    //get current position relative to initial position
    Translation2d currentRelPosition = 
      RobotContainer.swerveDrive.getCurrentPose().getTranslation();
    Translation2d currentRelVelocity = 
      RobotContainer.swerveDrive.getCurrentVelocity().getTranslation();

    double awayOutput = awayPosPidController.calculate(currentRelPosition.getX(), targetPosX)+
      awaySpeedPIDController.calculate(currentRelVelocity.getX(), targetVelX) + 
      awaySpeedFeedforward.calculate(targetVelX, targetAccX);
    
    double lateralOutput = lateralPosPidController.calculate(currentRelPosition.getY(), targetPosY)+
      lateralSpeedPIDController.calculate(currentRelVelocity.getY(), targetVelY) + 
      lateralSpeedFeedforward.calculate(targetVelY, targetAccY);
    

    //convert back to field centric drive speeds
    RobotContainer.swerveDrive.driveFieldCentric(awayOutput, lateralOutput,
      RobotContainer.swerveDrive.getRobotRotationPIDOut(targetAngle), kDriveMode.percentOutput);
    
    System.out.println("CV:," + currentRelVelocity.getX() 
      +",GV:,"+targetVelX+",dV:,"+(currentRelVelocity.getX()-targetVelX));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.swerveDrive.stopAllModules();
    RobotContainer.swerveDrive.setOdometryActive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

}
