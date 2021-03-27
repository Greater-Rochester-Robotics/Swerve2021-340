// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveStraightTrapProfile extends CommandBase {
  private TrapezoidProfile profile;
  private Timer timer = new Timer();
  private Translation2d initialPosition;
  private double angleOfRobot;
  private Rotation2d directionAsAngle;

  /**
   * Drives the robot a given direction, as given by an angle 
   * in radians from the x axis of the field, a distance per 
   * distanceOfTravel in meters, starting at speed startSpeed, 
   * and ending with speed endSpeed.
   * 
   * @param directionAsAngle angle in radians from the positive x-axis of thhe field
   * @param distanceOfTravel a distance in meters the robot will travel
   * @param startSpeed a speed
   * @param endSpeed
   */
  public DriveStraightTrapProfile(double directionAsAngle,double distanceOfTravel, double startSpeed, double endSpeed) {
    addRequirements(RobotContainer.swerveDrive);
    
    TrapezoidProfile.State start = new TrapezoidProfile.State(0,startSpeed);
    TrapezoidProfile.State target = new TrapezoidProfile.State(distanceOfTravel,endSpeed);

    //generate a trapezoidal profile for driving a straight line
    profile = new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(4.0, 2.0),
            // Goal state
            target,
            // Initial state
            start);

    this.directionAsAngle = new Rotation2d(directionAsAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleOfRobot = RobotContainer.swerveDrive.getGyroInRad();

    initialPosition = RobotContainer.swerveDrive.getCurrentPose().getTranslation();
    timer.reset();
    timer.start();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //use profile to create a position and speed for the motors
    TrapezoidProfile.State currentPoint = profile.calculate(timer.get());
    TrapezoidProfile.State futurePoint = profile.calculate(timer.get() + 0.02);
    double acceleration = (futurePoint.velocity - currentPoint.velocity) / 0.02;
    // System.out.println("position:" + currentPoint.position);
    //get current position relative to initial position
    Translation2d currentRelPostion = 
      RobotContainer.swerveDrive.getCurrentPose().getTranslation().minus(initialPosition);
    Translation2d currentRelVelocity = 
      RobotContainer.swerveDrive.getCurrentVelocity().getTranslation();

    //get the position in drive orientation, uses directionAsAngle
    Translation2d currentDrivePosition = currentRelPostion.rotateBy(directionAsAngle.unaryMinus());
    Translation2d currentDriveVelocity = currentRelVelocity.rotateBy(directionAsAngle.unaryMinus());

    //use PID controller to get drive orientation outputs
    double movingDirection = /*RobotContainer.swerveDrive.awayPosPidController.calculate(
      currentDrivePosition.getX(), currentPoint.position) + */
      RobotContainer.swerveDrive.awaySpeedFeedforward.calculate(currentPoint.velocity, acceleration);//position or velocity pid
    
    // double movingDirection = RobotContainer.swerveDrive.awaySpeedPIDController.calculate(
    //   currentDriveVelocity.getX(), currentPoint.velocity) + 
    //   RobotContainer.swerveDrive.awaySpeedFeedforward.calculate(currentPoint.velocity, acceleration);//position or velocity pid
    
    double notMovingDirection = RobotContainer.swerveDrive.lateralPosPidController.calculate(
      currentDrivePosition.getY(), 0.0);//Position PID
    // System.out.println("Current Position:" + currentDrivePosition.getX());
    
    Translation2d output = new Translation2d(movingDirection, notMovingDirection).rotateBy(directionAsAngle);
    
    // System.out.println("output x:" + output.getX());

    //convert back to field centric drive speeds
    RobotContainer.swerveDrive.driveFieldCentric(output.getX(), output.getY(),
      RobotContainer.swerveDrive.getRobotRotationPIDOut(angleOfRobot), kDriveMode.percentOutput);
    
    System.out.println("Current Velocity:" + RobotContainer.swerveDrive.getCurrentVelocity().getX() 
      +"  goal Velo:"+currentPoint.velocity);
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
    return timer.hasElapsed(profile.totalTime());
  }

  public double distanceBetweenPose(Pose2d a,Pose2d b){
    return Math.sqrt( Math.pow(a.getX()+b.getX(),2) + Math.pow(a.getY()+b.getY(),2) );
  }
}
