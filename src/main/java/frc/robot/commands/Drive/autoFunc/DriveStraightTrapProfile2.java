// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveStraightTrapProfile2 extends CommandBase {
  private TrapezoidProfile profile;
  private Timer timer = new Timer();
  private Translation2d initialPosition;
  private double angleOfRobot;
  private Rotation2d directionAsAngle;

  private Translation2d currentDrivePosition;
  private Translation2d currentDriveVelocity;
  private double distanceOfTravel;

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
  public DriveStraightTrapProfile2(double directionAsAngle, double distanceOfTravel, double startSpeed, double endSpeed) {
    addRequirements(RobotContainer.swerveDrive);
    this.distanceOfTravel = distanceOfTravel;
    
    TrapezoidProfile.State start = new TrapezoidProfile.State(0,startSpeed);
    TrapezoidProfile.State target = new TrapezoidProfile.State(distanceOfTravel,endSpeed);

    //generate a trapezoidal profile for driving a straight line
    profile = new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(4.0, 4.0),
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
    System.out.println("direction start" + this.directionAsAngle);
    initialPosition = RobotContainer.swerveDrive.getCurrentPose().getTranslation();
    timer.reset();
    timer.start();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //use profile to create a position and speed for the motors
    TrapezoidProfile.State targetPoint = profile.calculate(timer.get()+.02);
    double acceleration = (profile.calculate(timer.get()+.01).velocity
      - profile.calculate(timer.get()-.01).velocity)/.02;
    
    //get current position relative to initial position
    Translation2d currentRelPostion = 
      RobotContainer.swerveDrive.getCurrentPose().getTranslation().minus(initialPosition);
    Translation2d currentRelVelocity = 
      RobotContainer.swerveDrive.getCurrentVelocity().getTranslation();

    //get the position in drive orientation, uses directionAsAngle
    currentDrivePosition = currentRelPostion.rotateBy(directionAsAngle.unaryMinus());
    currentDriveVelocity = currentRelVelocity.rotateBy(directionAsAngle.unaryMinus());

    //use PID controller to get drive orientation outputs
    // double movingDirection = //RobotContainer.swerveDrive.awayPosPidController.calculate(
      // currentDrivePosition.getX(), targetPoint.position); //+ 
    //  RobotContainer.swerveDrive.awaySpeedFeedforward.calculate(targetPoint.velocity, acceleration);//position or velocity pid
    
    double movingDirection = RobotContainer.swerveDrive.awayPosPidController.calculate(
      currentDrivePosition.getX(), targetPoint.position)+
      RobotContainer.swerveDrive.awaySpeedPIDController.calculate(
      currentDriveVelocity.getX(), targetPoint.velocity) + 
      RobotContainer.swerveDrive.awaySpeedFeedforward.calculate(targetPoint.velocity, acceleration);//position or velocity pid
    
    double notMovingDirection = RobotContainer.swerveDrive.lateralPosPidController.calculate(
      currentDrivePosition.getY(), 0.0);//Position PID
    // System.out.println("Current Position:" + currentDrivePosition.getX());
    
    Translation2d output = new Translation2d(movingDirection, notMovingDirection).rotateBy(directionAsAngle);
    
    // System.out.println("output x:" + output.getX());

    //convert back to field centric drive speeds
    RobotContainer.swerveDrive.driveFieldCentric(output.getX(), output.getY(),
      RobotContainer.swerveDrive.getRobotRotationPIDOut(angleOfRobot), kDriveMode.percentOutput);
    
    System.out.println("CV:," + currentDriveVelocity.getX() 
      +",GV:,"+targetPoint.velocity+",dV:,"+(currentDriveVelocity.getX()-targetPoint.velocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("distance stop "+distanceOfTravel+" at "+currentDrivePosition.getX());
    timer.stop();
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timer.hasElapsed(profile.totalTime());// || 
           // distanceOfTravel <= (currentDrivePosition.getX()+.05);//(currentDriveVelocity.getX() * 0.2)));
  }

}
